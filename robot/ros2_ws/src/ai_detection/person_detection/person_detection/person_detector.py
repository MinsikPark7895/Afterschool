#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from tf2_ros import TransformListener, Buffer, TransformException
import tf2_ros
import os
from datetime import datetime
import math
from nav_msgs.msg import Odometry
from ultralytics import YOLO

# --- S3 업로드 기능 추가 ---
import boto3
from botocore.exceptions import NoCredentialsError
# --- 여기까지 ---


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        
        self.image_subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.yolo_model = YOLO('yolov8n.pt')
        
        self.person_detected = False
        self.person_center_x = 0
        self.image_width = 320
        self.last_detection_time = self.get_clock().now()
        
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Person Detector Node Started')
        self.get_logger().info('Waiting for camera images on topic: /camera/image_raw')
        
        self.detection_times = []
        self.fps_count = 0
        self.fps_start_time = time.time()
        self.performance_timer = self.create_timer(5.0, self.report_performance)

        self.last_log_time = time.time()
        self.log_interval = 3.0

        self.save_images = False
        self.last_save_time = 0
        self.save_interval = 5.0
        self.image_counter = 0
        self.save_directory = "/home/minju/person_detection_images"
        os.makedirs(self.save_directory, exist_ok=True)
        
        # --- S3 업로드 기능 추가 ---
        # S3 버킷명은 환경 변수에서 가져오거나 기본값 사용
        self.s3_bucket_name = os.getenv('AWS_S3_BUCKET', 'afterschool-s3-bucket')
        # --- 여기까지 ---

        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

    # --- S3 업로드 기능 추가 ---
    def upload_to_s3(self, local_file_path, s3_object_name):
        """로컬 파일을 S3 버킷에 업로드"""
        s3_client = boto3.client('s3')
        try:
            s3_client.upload_file(local_file_path, self.s3_bucket_name, s3_object_name)
            self.get_logger().info(f"Successfully uploaded {s3_object_name} to S3 bucket {self.s3_bucket_name}")
            return True
        except FileNotFoundError:
            self.get_logger().error(f"The file {local_file_path} was not found for S3 upload")
            return False
        except NoCredentialsError:
            self.get_logger().error("AWS Credentials not available. Run 'aws configure'.")
            return False
        except Exception as e:
            self.get_logger().error(f"Failed to upload to S3: {e}")
            return False
    # --- 여기까지 ---

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
            results = self.yolo_model(image, verbose=False)
            person_found = False
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        cls = int(box.cls[0])
                        if cls == 0 and float(box.conf[0]) > 0.5:
                            x1, y1, x2, y2 = box.xyxy[0]
                            box_area = (x2 - x1) * (y2 - y1)
                            
                            self.person_center_x = int((x1 + x2) / 2)
                            self.person_detected = True
                            self.person_distance = box_area
                            self.last_detection_time = self.get_clock().now()
                            person_found = True

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
            self.get_logger().info(f"Perf Report: Avg={avg_time:.1f}ms, Max={max_time:.1f}ms, FPS={fps:.1f}")
            self.detection_times.clear()
            self.fps_count = 0
            self.fps_start_time = time.time()

    def save_image(self, image):
        try:
            if self.get_map_position():
                coord_type = "map"
                self.get_logger().info(f"Robot pos (MAP): x={self.robot_x:.3f}, y={self.robot_y:.3f}")
            else:
                coord_type = "odom"
                self.robot_x, self.robot_y, self.robot_theta = self.odom_x, self.odom_y, self.odom_theta
                self.get_logger().warn(f"Using odom: x={self.robot_x:.3f}, y={self.robot_y:.3f}")

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"person_{coord_type}_{timestamp}_x{self.robot_x:.2f}_y{self.robot_y:.2f}_th{self.robot_theta:.2f}.jpg"
            filepath = os.path.join(self.save_directory, filename)
            
            success = cv2.imwrite(filepath, image)
            if success:
                self.image_counter += 1
                self.get_logger().info(f"Image saved locally: {filename}")
                
                # --- S3 업로드 기능 추가 ---
                # 로컬 저장이 성공하면 S3에 업로드
                s3_object_name = f"test/{filename}" # S3 버킷 안의 images 폴더에 저장
                self.upload_to_s3(filepath, s3_object_name)
                # --- 여기까지 ---

            else:
                self.get_logger().error(f"Failed to write image: {filepath}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.odom_theta = math.atan2(siny_cosp, cosy_cosp)

    def get_map_position(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
            q = transform.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.robot_theta = math.atan2(siny_cosp, cosy_cosp)
            return True
        except TransformException as e:
            return False

    def control_loop(self):
        twist = Twist()
        current_time = time.time()
        
        if self.person_detected:
            STOP_DISTANCE, APPROACH_DISTANCE = 15000, 8000
            
            if hasattr(self, 'person_distance'):
                image_center = self.image_width // 2
                error = self.person_center_x - image_center
                normalized_error = error / (self.image_width / 2)
                
                if self.person_distance > STOP_DISTANCE:
                    twist.linear.x = -0.05
                    log_msg = f'Too close! Backing up. Dist: {self.person_distance:.0f}'
                elif self.person_distance < APPROACH_DISTANCE:
                    twist.linear.x = 0.1
                    twist.angular.z = -0.5 * normalized_error if abs(normalized_error) > 0.1 else 0.0
                    log_msg = f'Approaching. Dist: {self.person_distance:.0f}'
                else:
                    twist.angular.z = -0.3 * normalized_error if abs(normalized_error) > 0.1 else 0.0
                    log_msg = f'Good distance. Adjusting. Dist: {self.person_distance:.0f}'
                
                if current_time - self.last_log_time >= self.log_interval:
                    self.get_logger().info(log_msg)
        else:
            twist.angular.z = 0.3
            
        if current_time - self.last_log_time >= self.log_interval:
            self.last_log_time = current_time
            
        self.cmd_vel_publisher.publish(twist)

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

if __name__ == '__main__':
    main()