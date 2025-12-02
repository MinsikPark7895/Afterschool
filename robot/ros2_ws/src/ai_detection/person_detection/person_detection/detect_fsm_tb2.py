#!/usr/bin/env python3
import rclpy, time, os, math, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from datetime import datetime

class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')

        # -------------------- Parameters --------------------
        self.declare_parameter('ns', 'tb2')
        self.declare_parameter('image_topic', '/tb2/camera/image_raw')
        self.declare_parameter('fsm_state_topic', 'fsm/state')
        self.declare_parameter('fsm_cmd_topic',   'fsm/cmd')

        # 탐지/성능
        self.declare_parameter('conf', 0.25)              # ↓ 0.50 -> 0.25
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('imgsz', 640)              # YOLO 입력크기 고정
        self.declare_parameter('min_event_interval', 0.7) # ↓ 2.0 -> 1.0 (조금 민첩하게)
        self.declare_parameter('keep_alive_sec', 3.0)     # 최근 탐지 후 이 시간은 True 유지(플리커 방지)
        self.keep_alive = float(self.get_parameter('keep_alive_sec').value)

        # 로깅/저장
        self.declare_parameter('log_interval', 3.0)
        self.declare_parameter('save_interval', 5.0)
        self.declare_parameter('save_dir', '/home/jelly/person_detection_images')

        # -------------------- Resolve --------------------
        self.ns = self.get_parameter('ns').get_parameter_value().string_value
        self.image_topic     = self.get_parameter('image_topic').get_parameter_value().string_value
        self.fsm_state_topic = self._ns_join(self.get_parameter('fsm_state_topic').get_parameter_value().string_value)
        self.fsm_cmd_topic   = self._ns_join(self.get_parameter('fsm_cmd_topic').get_parameter_value().string_value)

        self.conf_thresh  = float(self.get_parameter('conf').value)
        self.iou_thresh   = float(self.get_parameter('iou').value)
        self.imgsz        = int(self.get_parameter('imgsz').value)
        self.min_ev_int   = float(self.get_parameter('min_event_interval').value)
        self.keep_alive   = float(self.get_parameter('keep_alive_sec').value)

        self.log_interval = float(self.get_parameter('log_interval').value)
        self.save_interval= float(self.get_parameter('save_interval').value)
        self.save_directory = self.get_parameter('save_dir').get_parameter_value().string_value

        # -------------------- Pub/Sub --------------------
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_cb, 10)
        self.pub_detected = self.create_publisher(Bool,  self._ns_join('person_detected'), 10)
        self.pub_pos      = self.create_publisher(Point, self._ns_join('person_position'), 10)
        self.sub_state    = self.create_subscription(String, self.fsm_state_topic, self.state_cb, 10)
        self.pub_fsm_cmd  = self.create_publisher(String, self.fsm_cmd_topic, 10)

        # -------------------- Runtime --------------------
        self.bridge = CvBridge()
        self.current_state = 'UNKNOWN'
        self.person_detected = False
        self.image_wh = (320, 240)
        self.last_seen_wall = 0.0
        self.last_ev_wall = 0.0
        self.last_log = time.time()

        # Perf
        self.det_times = []
        self.fps_n = 0
        self.fps_t0 = time.time()
        self.create_timer(5.0, self.report_perf)

        os.makedirs(self.save_directory, exist_ok=True)

        # YOLO lazy load
        self._model = None
        self._device = 'cpu'

        self.get_logger().info(
            f"PersonDetector started\n"
            f"- ns={self.ns}\n- image_topic={self.image_topic}\n"
            f"- conf={self.conf_thresh}, iou={self.iou_thresh}, imgsz={self.imgsz}\n"
            f"- keep_alive={self.keep_alive}s"
        )

    # -------------------- Utils --------------------
    def _ns_join(self, suffix: str) -> str:
        s = suffix.lstrip('/'); ns = self.ns.strip('/')
        return f'/{ns}/{s}' if ns else f'/{s}'

    def _ensure_model(self):
        if self._model is not None: return
        from ultralytics import YOLO
        try:
            # 디바이스 선택
            try:
                import torch
                self._device = 'cuda' if torch.cuda.is_available() else 'cpu'
            except Exception:
                self._device = 'cpu'
            self.get_logger().info(f"[YOLO] Loading yolov8n.pt on {self._device} ...")
            self._model = YOLO('yolov8n.pt')
            self.get_logger().info("[YOLO] Model loaded.")
        except Exception as e:
            self.get_logger().error(f"[YOLO] load failed: {e}")
            raise

    # -------------------- Callbacks --------------------
    def state_cb(self, msg: String):
        self.current_state = msg.data.strip().upper()
        self.get_logger().info(f"[FSM] state: {self.current_state}")
        if self.current_state not in ['PATROLLING', 'TRACKING']:
            # 상태 벗어나면 탐지 False 브로드캐스트
            self._broadcast_detect(False)

    def image_cb(self, msg: Image):
        if self.current_state not in ['PATROLLING', 'TRACKING']:
            return

        t0 = time.time()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"[CvBridge] {e}")
            self._broadcast_detect(False)
            return

        H, W = frame.shape[:2]
        self.image_wh = (W, H)

        # ---- YOLO inference ----
        self._ensure_model()
        try:
            results = self._model.predict(
                source=frame,
                conf=self.conf_thresh,
                iou=self.iou_thresh,
                imgsz=self.imgsz,
                classes=[0],        # person
                device=self._device,
                verbose=False,
                stream=False,
                half=(self._device=='cuda'),
            )
        except Exception as e:
            self.get_logger().error(f"[YOLO] inference error: {e}")
            self._broadcast_detect(False)
            return

        # ---- parse best person ----
        best = None
        best_conf = 0.0
        import numpy as np
        r0 = results[0]
        boxes = getattr(r0, 'boxes', None)
        if boxes is not None and len(boxes) > 0:
            xyxy = boxes.xyxy.cpu().numpy()
            conf = boxes.conf.cpu().numpy()
            cls  = boxes.cls.cpu().numpy().astype(int)
            for i in range(len(xyxy)):
                if cls[i] != 0: 
                    continue
                if conf[i] < self.conf_thresh:
                    continue
                x1,y1,x2,y2 = xyxy[i]
                area = max(0.0, (x2-x1)*(y2-y1))
                if conf[i] > best_conf:
                    best_conf = float(conf[i])
                    best = (x1,y1,x2,y2,area)

        now = time.time()
        if best is not None:
            x1,y1,x2,y2,area = best
            cx = (x1+x2)/2.0
            area_ratio = area / float(W*H)  # 0.0 ~ 1.0

            # 상태 업데이트 + 히스테리시스
            self.person_detected = True
            self.last_seen_wall = now

            # 디버그
            if now - self.last_log >= self.get_parameter('log_interval').value:
                self.get_logger().info(
                    f"[DETECT] FOUND conf={best_conf:.2f}, cx={int(cx)}, "
                    f"area={int(area)}, ratio={area_ratio:.4f}"
                )
                self.last_log = now

            # 주제 발행: x=center_x, y=area(px), z=area_ratio(0~1)
            p = Point()
            p.x = float(cx)
            p.y = float(area)
            p.z = float(area_ratio)
            self.pub_pos.publish(p)
            self._broadcast_detect(True)

            # PATROLLING에서만 EV_DETECTED (디바운스)
            if self.current_state == 'PATROLLING' and (now - self.last_ev_wall) >= self.min_ev_int:
                self.pub_fsm_cmd.publish(String(data='EV_DETECTED'))
                self.last_ev_wall = now
                self.get_logger().info("[FSM] Published EV_DETECTED.")

            # 이미지 저장 (5초 간격)
            if not hasattr(self, 'last_save_time'):
                self.last_save_time = 0.0
            
            if now - self.last_save_time >= self.save_interval:
                self.save_image_with_s3(frame, best)
                self.last_save_time = now

        else:
            # 플리커 방지: 최근 keep_alive 내면 True 유지
            if (now - self.last_seen_wall) <= self.keep_alive:
                self._broadcast_detect(True)
            else:
                self._broadcast_detect(False)

        # perf
        self.det_times.append((time.time()-t0)*1000.0)
        self.fps_n += 1

    # -------------------- helpers --------------------
    def _broadcast_detect(self, is_true: bool):
        if is_true != self.person_detected:
            # 상태 전환에서만 Info 로그
            self.get_logger().info(f"[DETECT] {'TRUE' if is_true else 'FALSE'}")
        self.person_detected = is_true
        self.pub_detected.publish(Bool(data=is_true))

    def report_perf(self):
        if not self.det_times: return
        avg = sum(self.det_times)/len(self.det_times)
        mn, mx = min(self.det_times), max(self.det_times)
        elapsed = time.time() - self.fps_t0
        fps = self.fps_n/elapsed if elapsed>0 else 0.0
        self.get_logger().info(
            f"=== PERF === avg {avg:.1f} ms, min/max {mn:.1f}/{mx:.1f} ms, fps {fps:.1f}"
        )
        self.det_times.clear(); self.fps_n = 0; self.fps_t0 = time.time()

    # (이미지 저장/TF는 그대로 필요 시 추가)
    def save_image_with_s3(self, frame, best_detection=None):
        """이미지를 S3에 업로드"""
        try:
            import boto3
            from botocore.exceptions import ClientError
            
            # 맵 좌표 얻기 (기존 TF 코드 필요시 추가)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # 탐지 정보 포함한 파일명
            if best_detection:
                x1, y1, x2, y2, area = best_detection
                cx = int((x1 + x2) / 2)
                filename = f"person_{self.ns}_{timestamp}_cx{cx}_area{int(area)}.jpg"
            else:
                filename = f"person_{self.ns}_{timestamp}.jpg"
            
            # 임시 로컬 파일로 저장
            local_path = f"/tmp/{filename}"
            success = cv2.imwrite(local_path, frame)
            
            if success:
                try:
                    # S3 업로드
                    s3_client = boto3.client('s3')
                    s3_key = f"evidence/{filename}"
                    
                    s3_bucket = os.getenv('AWS_S3_BUCKET', 'afterschool-s3-bucket')
                    s3_client.upload_file(
                        local_path,
                        s3_bucket, 
                        s3_key
                    )
                    
                    self.get_logger().info(f"[S3] Uploaded: s3://{s3_bucket}/{s3_key}")
                    
                    # 로컬 임시 파일 삭제
                    os.remove(local_path)
                    return True
                    
                except ClientError as e:
                    self.get_logger().error(f"[S3] Upload failed: {e}")
                    # S3 실패 시 로컬에 보관
                    local_backup = os.path.join(self.save_directory, filename)
                    os.rename(local_path, local_backup)
                    self.get_logger().info(f"[S3] Saved locally instead: {local_backup}")
                    return False
            else:
                self.get_logger().error(f"[S3] Failed to save image locally")
                return False
                
        except Exception as e:
            self.get_logger().error(f"[S3] Error: {e}")
            return False    

def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

