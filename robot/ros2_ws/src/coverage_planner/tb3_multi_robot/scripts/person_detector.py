#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
from tf2_ros import TransformListener, Buffer, TransformException
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        
        # 구독자 설정
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # 발행자 설정 - 사람 탐지 상태 발행
        self.detection_publisher = self.create_publisher(Bool, '/person_detected', 10)
        self.person_position_publisher = self.create_publisher(Point, '/person_position', 10)
        
        # OpenCV 설정
        self.bridge = CvBridge()
        
        # 탐지 관련 변수
        self.person_detected = False
        self.person_center_x = 0
        self.image_width = 320
        self.last_detection_time = self.get_clock().now()
        
        # 성능 측정용 변수들
        self.detection_times = []
        self.fps_count = 0
        self.fps_start_time = time.time()
        
        # 성능 리포트 타이머
        self.performance_timer = self.create_timer(5.0, self.report_performance)

        # 로그 제어용 변수
        self.last_log_time = time.time()
        self.log_interval = 3.0
        
        # 이미지 저장 관련 변수
        self.save_images = False
        self.last_save_time = 0
        self.save_interval = 5.0
        self.image_counter = 0
        self.save_directory = "/home/minju/teto/person_detection_images"
        
        # 저장 디렉토리 생성
        import os
        os.makedirs(self.save_directory, exist_ok=True)

        # tf2 설정 (절대좌표 변환용)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 로봇 위치 변수
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        self.get_logger().info('Person Detector Node Started (Detection Only Mode)')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_width = cv_image.shape[1]
            self.detect_person(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {e}')
            self.person_detected = False

    def detect_person(self, image):
        detection_start = time.time()
        
        try:
            from ultralytics import YOLO
            
            if not hasattr(self, 'yolo_model'):
                self.yolo_model = YOLO('yolov8n.pt')
            
            results = self.yolo_model(image, verbose=False)
            person_found = False
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        cls = int(box.cls[0])
                        conf = float(box.conf[0])
                        
                        if cls == 0 and conf > 0.5:
                            x1, y1, x2, y2 = box.xyxy[0]
                            box_area = (x2 - x1) * (y2 - y1)
                            
                            self.person_center_x = int((x1 + x2) / 2)
                            self.person_detected = True
                            self.person_distance = box_area
                            self.last_detection_time = self.get_clock().now()
                            person_found = True

                            # 탐지 상태 발행
                            detection_msg = Bool()
                            detection_msg.data = True
                            self.detection_publisher.publish(detection_msg)
                            
                            # 사람 위치 발행 (이미지 좌표계)
                            position_msg = Point()
                            position_msg.x = float(self.person_center_x)
                            position_msg.y = float(box_area)
                            position_msg.z = 0.0
                            self.person_position_publisher.publish(position_msg)

                            # 이미지 저장
                            if not self.save_images:
                                self.save_images = True
                                self.last_save_time = time.time()
                                self.get_logger().info("Person detected! Starting image capture...")
                            
                            current_time = time.time()
                            if current_time - self.last_save_time >= self.save_interval:
                                self.save_image(image)
                                self.last_save_time = current_time

                            return
            
            if not person_found:
                current_time = self.get_clock().now()
                if (current_time - self.last_detection_time).nanoseconds / 1e9 > 2.0:
                    self.person_detected = False
                    
                    # 미탐지 상태 발행
                    detection_msg = Bool()
                    detection_msg.data = False
                    self.detection_publisher.publish(detection_msg)

                    if self.save_images:
                        self.save_images = False
                        self.get_logger().info("Person lost! Stopping image capture.")
                        
        except Exception as e:
            self.get_logger().error(f'YOLO detection error: {e}')
            self.person_detected = False
        finally:
            detection_time = (time.time() - detection_start) * 1000
            self.detection_times.append(detection_time)
            self.fps_count += 1
 
    def report_performance(self):
        if len(self.detection_times) > 0:
            avg_time = sum(self.detection_times) / len(self.detection_times)
            max_time = max(self.detection_times)
            min_time = min(self.detection_times)
            
            elapsed = time.time() - self.fps_start_time
            fps = self.fps_count / elapsed if elapsed > 0 else 0
            
            self.get_logger().info(f"""
=== PERFORMANCE REPORT ===
Average Detection Time: {avg_time:.1f}ms
Min/Max Time: {min_time:.1f}/{max_time:.1f}ms
Processing FPS: {fps:.1f}
Frames Processed: {self.fps_count}
=========================""")
            
            self.detection_times.clear()
            self.fps_count = 0

    def save_image(self, image):
        try:
            import os
            from datetime import datetime

            if self.get_map_position():
                coord_type = "map"
                self.get_logger().info(f"Robot position (MAP coordinate): x={self.robot_x:.3f}, y={self.robot_y:.3f}, theta={self.robot_theta:.3f}")
            else:
                coord_type = "no_map"
                self.get_logger().warn("Could not get MAP position")

            if not os.path.exists(self.save_directory):
                os.makedirs(self.save_directory, exist_ok=True)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"person_{coord_type}_{timestamp}_x{self.robot_x:.2f}_y{self.robot_y:.2f}_th{self.robot_theta:.2f}.jpg"
            filepath = os.path.join(self.save_directory, filename)
            
            success = cv2.imwrite(filepath, image)
            if success:
                self.image_counter += 1
                self.get_logger().info(f"Image saved: {filename}")
            else:
                self.get_logger().error(f"Failed to write image: {filepath}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")

    def get_map_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', 
                rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
            
            import math
            q = transform.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.robot_theta = math.atan2(siny_cosp, cosy_cosp)
            
            return True
            
        except:
            return False

    def __del__(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        person_detector = PersonDetector()
        rclpy.spin(person_detector)
    except KeyboardInterrupt:
        pass
    finally:
        if 'person_detector' in locals():
            person_detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
