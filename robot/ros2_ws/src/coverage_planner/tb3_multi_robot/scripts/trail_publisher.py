#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class TrailPublisher(Node):
    def __init__(self):
        super().__init__('trail_publisher', namespace='')
        self.declare_parameter('robot_id', 'tb1')
        self.robot_id = self.get_parameter('robot_id').value
        self.points = []
        self.sub = self.create_subscription(
            Odometry, f'{self.get_namespace()}/odom', self.odom_callback, 10)
        self.pub = self.create_publisher(Marker, 'gazebo_trail', 10)
        self.timer = self.create_timer(1.0, self.check_odom)
        self.odom_received = False
        self.get_logger().info(f'{self.robot_id} TrailPublisher started')

    def check_odom(self):
        if not self.odom_received:
            self.get_logger().warn(f'{self.robot_id} Waiting for /odom data')

    def odom_callback(self, msg):
        self.odom_received = True
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.points.append(Point(x=x, y=y, z=0.01))
        if len(self.points) > 10000:
            self.points.pop(0)

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.robot_id
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0 if self.robot_id == 'tb1' else 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0 if self.robot_id == 'tb1' else 1.0
        marker.color.a = 1.0
        marker.points = self.points
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        self.pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = TrailPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down TrailPublisher')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()