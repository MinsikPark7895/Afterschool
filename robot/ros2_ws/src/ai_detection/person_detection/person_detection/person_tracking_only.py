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
        self.image_width = 1920  # 기본값
        
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
            # 바운딩 박스를 이미지 크기로 정규화
            total_pixels = self.image_width * 1080  # height는 1080 고정
            box_ratio = self.person_distance / total_pixels
            
            # 정규화된 임계값 사용
            STOP_RATIO = 0.3    # 화면의 30% 이상이면 너무 가까움
            APPROACH_RATIO = 0.2  # 화면의 20% 이하면 더 가까이
            
            if box_ratio > STOP_RATIO:
                # 너무 가까움 - 후진
                twist.linear.x = -0.4
                twist.angular.z = 0.0
                self.get_logger().info(f'너무 가까움. Box ratio: {box_ratio:.4f}')
            
            elif box_ratio < APPROACH_RATIO:
                # 충분히 멀음 - 전진하면서 사람 추적
                image_center = self.image_width // 2
                error = self.person_center_x - image_center
                normalized_error = error / (self.image_width / 2)
                
                twist.linear.x = 0.4  # 전진 속도
                
                if abs(normalized_error) > 0.15:
                    angular_speed = -0.05 * normalized_error
                    angular_speed = max(-0.2, min(0.2, angular_speed))
                    twist.angular.z = angular_speed
                    self.get_logger().info(f'전진+회전. Box ratio: {box_ratio:.4f}, Error: {normalized_error:.3f}')
                else:
                    twist.angular.z = 0.0
                    self.get_logger().info(f'전진중. Box ratio: {box_ratio:.4f}')
            
            else:
                # 적정 거리 - 사람을 중앙에 맞추기만 함
                image_center = self.image_width // 2
                error = self.person_center_x - image_center
                normalized_error = error / (self.image_width / 2)
                
                if abs(normalized_error) > 0.15:
                    angular_speed = -0.05 * normalized_error
                    angular_speed = max(-0.2, min(0.2, angular_speed))
                    twist.angular.z = angular_speed
                    self.get_logger().info(f'각도조정. Box ratio: {box_ratio:.4f}, Error: {normalized_error:.3f}')
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info(f'완벽! Box ratio: {box_ratio:.4f}')
        
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