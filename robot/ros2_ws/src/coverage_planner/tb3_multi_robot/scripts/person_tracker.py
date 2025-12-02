#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool

class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')
        
        # 구독자 설정
        self.detection_subscription = self.create_subscription(
            Bool,
            '/person_detected',
            self.detection_callback,
            10)
        
        self.position_subscription = self.create_subscription(
            Point,
            '/person_position',
            self.position_callback,
            10)
        
        # 발행자 설정
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 제어 관련 변수
        self.person_detected = False
        self.person_center_x = 0
        self.person_distance = 0
        self.image_width = 320  # 기본값
        
        # 타이머 설정 (제어 루프)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Person Tracker Node Started')

    def detection_callback(self, msg):
        """사람 탐지 상태 수신"""
        self.person_detected = msg.data

    def position_callback(self, msg):
        """사람 위치 정보 수신"""
        self.person_center_x = int(msg.x)
        self.person_distance = msg.y

    def control_loop(self):
        """로봇 제어 루프 - 사람 추적"""
        twist = Twist()
        
        if self.person_detected:
            # 거리 임계값 설정 (바운딩 박스 면적 기준)
            STOP_DISTANCE = 15000  # 이 값 이상이면 너무 가까움
            APPROACH_DISTANCE = 8000  # 이 값 이하면 더 가까이 가기
            
            if self.person_distance > STOP_DISTANCE:
                # 너무 가까움 - 후진
                twist.linear.x = -0.3
                twist.angular.z = 0.0
                self.get_logger().info(f'Too close! Backing up. Distance: {self.person_distance:.0f}')
            
            elif self.person_distance < APPROACH_DISTANCE:
                # 충분히 멀음 - 전진하면서 사람 추적
                image_center = self.image_width // 2
                error = self.person_center_x - image_center
                normalized_error = error / (self.image_width / 2)
                
                if abs(normalized_error) > 0.1:
                    twist.angular.z = -0.5 * normalized_error
                    twist.linear.x = 0.5
                    self.get_logger().info(f'Approaching person, turning. Distance: {self.person_distance:.0f}')
                else:
                    twist.linear.x = 0.3
                    self.get_logger().info(f'Approaching person, moving forward. Distance: {self.person_distance:.0f}')
            
            else:
                # 적정 거리 - 사람을 중앙에 맞추기만 함
                image_center = self.image_width // 2
                error = self.person_center_x - image_center
                normalized_error = error / (self.image_width / 2)
                
                if abs(normalized_error) > 0.1:
                    twist.angular.z = -0.3 * normalized_error
                    self.get_logger().info(f'Good distance, adjusting angle. Distance: {self.person_distance:.0f}')
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info(f'Perfect distance and position! Distance: {self.person_distance:.0f}')
        
        else:
            # 사람이 감지되지 않으면 정지
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        person_tracker = PersonTracker()
        rclpy.spin(person_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        if 'person_tracker' in locals():
            person_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
