#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Empty
from rclpy.action.client import ClientGoalHandle
import math

def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class CoveragePlanner(Node):
    """Robot control loop with proper navigation handling"""
    def __init__(self):
        super().__init__('coverage_planner', namespace='')
        self.declare_parameter('spawn_x', '2.30')
        self.declare_parameter('spawn_y', '13.0')
        self.declare_parameter('spawn_yaw', '-1.5')
        self.declare_parameter('robot_id', 'tb1')
        self.declare_parameter('use_sim_time', True)

        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)
        self.spawn_yaw = float(self.get_parameter('spawn_yaw').value)

        # Define hard-coded patrol points (x, y, yaw)
        self.patrol_points = {
            # 'tb1': [[-48.5, -3.5, 0.0], [48.5, -3.5, 0.0]]
            # 'tb1': [
            #     [-2.3, 13.0, -1.5],
            #     [-2.3, 12.0, -1.5],
            #     [-6.5, 12.0, 3.0],
            #     [-6.5, 5.5, -1.5],
            #     [-2.3, 5.5, 0.0],
            #     [-2.3, 0.5, -1.5],
            #     [-6.5, 0.5, 3.0],
            #     [-6.5, -3.5, -1.5],
            #     [-48.5, -3.5, 3.0],
            #     [-48.5, -10.3, -1.5],
            #     [-48.5, -3.5, 1.5],
            #     [-6.5, -3.5, 0.0],
            #     [-6.5, -8.0, -1.5],
            #     [-2.3, -8.0, 0.0],
            #     [-2.3, -14.0, -1.5],
            #     [-6.5, -14.0, 3.0],
            #     [-2.3, -14.0, 0.0],
            #     [-2.3, -8.0, 1.5],
            #     [-6.5, -8.0, 3.0],
            #     [-6.5, 5.5, 1.5],
            #     [-2.3, 5.5, 0.0],
            #     [-2.3, 5.5, -1.5],
            #     [-6.5, 12.0, 2.3],
            #     [-2.3, 12.0, 0.0]
            # ],
            # 'tb2': [
            #     [2.3, 13.0, -1.5],
            #     [2.3, 12.0, -1.5],
            #     [6.5, 12.0, 0.0],
            #     [2.3, 5.5, -2.3],
            #     [2.3, 5.5, 3.0],
            #     [2.3, 0.5, -1.5],
            #     [6.5, 0.5, 0.0],
            #     [6.5, 0.5, 1.5],
            #     [2.3, -8.0, -2.3],
            #     [2.3, -8.0, 3.0],
            #     [2.3, -14.0, -1.5],
            #     [6.5, -14.0, 0.0],
            #     [6.5, -8.0, 1.5],
            #     [6.5, -8.0, 3.0],
            #     [6.5, -3.5, 1.5],
            #     [48.5, -3.5, 0.0],
            #     [48.5, -10.3, -1.5],
            #     [48.5, -3.5, 1.5],
            #     [6.5, -3.5, 3.0],
            #     [6.5, 0.5, 1.5],
            #     [6.5, 5.5, 7.5],
            #     [2.3, 5.5, 3.0],
            #     [2.3, 3.0, -1.5],
            #     [6.5, 5.5, 7.5],
            #     [6.5, 12.0, 1.5],
            #     [2.3, 12.0, 3.0],
            # ]


            'tb1': [
                [3.79, 20.62, -1.50],
                [3.79, 20.58, -1.50],
                [3.57, 20.58, 3.00],
                [3.57, 20.25, -1.50],
                [3.79, 20.25, 0.00],
                [3.79, 20.00, -1.50],
                [3.57, 20.00, 3.00],
                [3.57, 19.80, -1.50],
                [1.47, 19.80, 3.00],
                [1.47, 19.46, -1.50],
                [1.47, 19.80, 1.50],
                [3.57, 19.80, 0.00],
                [3.57, 19.58, -1.50],
                [3.79, 19.58, 0.00],
                [3.79, 19.28, -1.50],
                [3.57, 19.28, 3.00],
                [3.79, 19.28, 0.00],
                [3.79, 19.58, 1.50],
                [3.57, 19.58, 3.00],
                [3.57, 20.25, 1.50],
                [3.79, 20.25, 0.00],
                [3.79, 20.25, -1.50],
                [3.57, 20.58, 2.30],
                [3.79, 20.58, 0.00],
            ],
            'tb2': [
                [4.01, 20.62, -1.50],
                [4.01, 20.58, -1.50],
                [4.22, 20.58, 0.00],
                [4.01, 20.25, -2.30],
                [4.01, 20.25, 3.00],
                [4.01, 20.00, -1.50],
                [4.22, 20.00, 0.00],
                [4.22, 20.00, 1.50],
                [4.01, 19.58, -2.30],
                [4.01, 19.58, 3.00],
                [4.01, 19.28, -1.50],
                [4.22, 19.28, 0.00],
                [4.22, 19.58, 1.50],
                [4.22, 19.58, 3.00],
                [4.22, 19.80, 1.50],
                [6.33, 19.80, 0.00],
                [6.33, 19.46, -1.50],
                [6.33, 19.80, 1.50],
                [4.22, 19.80, 3.00],
                [4.22, 20.00, 1.50],
                [4.22, 20.25, 7.50],
                [4.01, 20.25, 3.00],
                [4.01, 20.12, -1.50],
                [4.22, 20.25, 7.50],
                [4.22, 20.58, 1.50],
                [4.01, 20.58, 3.00],
            ]
        }.get(self.robot_id, [])

        if not self.patrol_points:
            self.get_logger().error(f'{self.robot_id} No patrol points defined, shutting down')
            raise RuntimeError('No patrol points defined')

        self.current_waypoint_index = 0
        self.is_returning_to_spawn = False

        # Publishers and subscribers
        self.path_pub = self.create_publisher(Path, 'plan', 5)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.return_spawn_sub = self.create_subscription(Empty, 'return_to_spawn', self.return_to_spawn_callback, 10)
        self.get_logger().info(f'{self.robot_id} CoveragePlanner started with {len(self.patrol_points)} patrol points')
        self.timer = self.create_timer(1.0, self.execute_coverage)

    def return_to_spawn_callback(self, msg):
        self.is_returning_to_spawn = True
        self.current_waypoint_index = 0
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.spawn_x
        goal_msg.pose.pose.position.y = self.spawn_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(self.spawn_yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(self.spawn_yaw / 2.0)
        self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)
        self.get_logger().info(f'{self.robot_id} Returning to spawn: x={self.spawn_x}, y={self.spawn_y}')

    def execute_coverage(self):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'{self.robot_id} NavigateToPose action server not available at {self.get_namespace()}/navigate_to_pose')
            return

        if self.is_returning_to_spawn:
            return  # Wait until spawn goal is completed

        # Publish patrol path
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in self.patrol_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

        # Send current waypoint
        x, y, yaw = self.patrol_points[self.current_waypoint_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)
        self.get_logger().info(f'{self.robot_id} sent goal {self.current_waypoint_index + 1}/{len(self.patrol_points)}: x={x:.2f}, y={y:.2f}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{self.robot_id} Goal rejected')
            if not self.is_returning_to_spawn:
                self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.patrol_points)
            return
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().debug(f'{self.robot_id} Feedback received')

    def result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info(f'{self.robot_id} reached waypoint {self.current_waypoint_index + 1}')
            if self.is_returning_to_spawn:
                self.is_returning_to_spawn = False
                self.get_logger().info(f'{self.robot_id} Reached spawn, resuming patrol')
            else:
                self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.patrol_points)
        else:
            self.get_logger().error(f'{self.robot_id} Goal failed: {result.error_code}')
            if not self.is_returning_to_spawn:
                self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.patrol_points)

def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down CoveragePlanner')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()