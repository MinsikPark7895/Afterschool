#!/usr/bin/env python3
import time
import tf2_ros
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from std_msgs.msg import String
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
import numpy as np
if not hasattr(np, "float"):
    np.float = float
from tf_transformations import quaternion_from_euler

class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')
        # Declare and get waypoints parameter (list of [x, y] pairs)
        self.declare_parameter("waypoints", [0.0])
        raw_wps = self.get_parameter("waypoints").value
        self.waypoints = [(float(raw_wps[i]),
                        float(raw_wps[i+1]),
                        float(raw_wps[i+2]))
                        for i in range(0, len(raw_wps), 3)]

        # Convert flat list to list of (x,y) tuples, if necessary
        # (Assume user provided [x1,y1,x2,y2,...] for simplicity)
        ns = self.get_namespace().strip('/')
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints for namespace '{ns}'")

        # Setup ActionClient to Nav2 FollowWaypoints (server name 'follow_waypoints')
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # Internal state
        self._current_goal_handle = None
        self._paused = False
        self._returning = False

        # --- TF listener ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriber for external commands (e.g. "pause", "resume", "return")
        self.create_subscription(String, 'robot_command', self.command_callback, 10)

        # Start the patrol after a short delay to allow Nav2 to start up
        # self._start_timer = self.create_timer(1.0, self._start_patrol)

        # --- Start patrol after TF and Action server ready ---
        self.get_logger().info("Waiting for TF and Nav2 Action server...")
        self._startup_timer = self.create_timer(0.5, self.start_when_ready)

    def start_when_ready(self):
        tf_ok = self.tf_ready()
        action_ok = self.action_client.wait_for_server(timeout_sec=0.1)

        if not tf_ok:
            self.get_logger().info("Waiting for TF map->base_link...")
            return

        if not action_ok:
            self.get_logger().info("Waiting for Nav2 follow_waypoints action server...")
            return

        self.get_logger().info("TF and Nav2 Action server ready! Sending initial goal...")
        self.send_goal()
        self._startup_timer.cancel()


    def tf_ready(self):
        try:
            self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False


    # def wait_for_tf_ready(self, timeout_sec=60.0):
    #     self.get_logger().info("Waiting for map→base_link transform...")
    #     start = self.get_clock().now()
    #     while rclpy.ok():
    #         try:
    #             self.tf_buffer.lookup_transform(
    #                 "map",
    #                 "base_link",
    #                 rclpy.time.Time()
    #             )
    #             self.get_logger().info("Transform available!")
    #             return
    #         except tf2_ros.LookupException:
    #             pass
    #         except tf2_ros.ConnectivityException:
    #             pass
    #         except tf2_ros.ExtrapolationException:
    #             pass

    #         now = self.get_clock().now()
    #         if (now - start).nanoseconds / 1e9 > timeout_sec:
    #             self.get_logger().warn("Timeout waiting for transform!")
    #             break
    #     time.sleep(0.1)

    def _start_patrol(self):
        self.send_goal()
        # Cancel and destroy the timer to make it one-shot
        self._start_timer.cancel()
        self.destroy_timer(self._start_timer)

    def send_goal_once(self):
        if not self._paused and not self._returning:
            self.send_goal()

    def send_goal(self):
        if self._paused:
            self.get_logger().info('Patrol paused; not sending new goal.')
            return
        # Build FollowWaypoints goal: convert each (x,y) into a PoseStamped
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = []
        for x, y, theta in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            q = quaternion_from_euler(0, 0, theta)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            goal_msg.poses.append(pose)
        # Send the goal asynchronously
        # self.action_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info('Sent FollowWaypoints goal with %d waypoints' % len(self.waypoints))

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server, will retry in 1s')
            # One-shot retry timer
            timer = self.create_timer(1.0, self.send_goal_once)
            # destroy timer after first fire
            def destroy_timer_callback():
                timer.cancel()
                timer.destroy()
            timer.callback = lambda: (self.send_goal_once(), destroy_timer_callback())
            return

        self.get_logger().info('Goal accepted, moving robot...')
        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        current_index = feedback_msg.feedback.current_waypoint
        self.get_logger().debug(f'Currently heading to waypoint {current_index}')

    def result_callback(self, future):
        result = future.result().result
        missed = result.missed_waypoints
        if missed:
            self.get_logger().warn(f'Missed waypoints: {missed}')
        else:
            self.get_logger().info('All waypoints reached successfully')
        # Decide next action after finishing current goal
        if self._returning:
            self.get_logger().info('Returned to spawn; patrol halted.')
            return
        # Immediately send the same waypoint list again (continuous patrol)
        self.send_goal()

    def command_callback(self, msg):
        cmd = msg.data.lower()
        if cmd == 'pause':
            self._paused = True
            self.get_logger().info('Pause command received')
            # Cancel current goal if any
            # if self._current_goal_handle is not None:
            if self._current_goal_handle:
                self._current_goal_handle.cancel_goal_async()
        elif cmd == 'resume':
            if self._paused:
                self._paused = False
                self.get_logger().info('Resume command received')
                # If no goal active, send a new one
                if self._current_goal_handle:
                    self.send_goal()
        elif cmd == 'return':
            self.get_logger().info('Return-to-spawn command received')
            self._paused = False
            self._returning = True
            # Cancel current goal and send robot back to spawn (e.g., waypoint[0])
            if self._current_goal_handle:
                self._current_goal_handle.cancel_goal_async()
            # Here we assume the first waypoint is the spawn or you could set a separate spawn point
            if self.waypoints:
                spawn_x, spawn_y, _ = self.waypoints[0]
                self.send_return_goal(spawn_x, spawn_y)

    def send_return_goal(self, x, y):
        # Send a single waypoint goal to return to spawn
        goal_msg = FollowWaypoints.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        goal_msg.poses = [pose]
        # self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
