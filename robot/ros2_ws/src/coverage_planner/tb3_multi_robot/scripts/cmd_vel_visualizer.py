#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

class CmdVelVisualizer(Node):
    def __init__(self):
        super().__init__('cmd_vel_visualizer')
        self.declare_parameter('robot_id', 'tb1')
        robot_id = self.get_parameter('robot_id').value
        self.cmd_vel_sub = self.create_subscription(
            Twist, f'/{robot_id}/cmd_vel', self.cmd_vel_callback, 10)
        self.marker_pub = self.create_publisher(
            Marker, f'/{robot_id}/cmd_vel_marker', 10)
        self.get_logger().info(f'CmdVelVisualizer started for {robot_id}')

    def cmd_vel_callback(self, msg):
        marker = Marker()
        marker.header = Header(frame_id='base_link', stamp=self.get_clock().now().to_msg())
        marker.ns = 'cmd_vel'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(min(msg.linear.x, 1.0), -1.0)  # Arrow length = linear.x
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()