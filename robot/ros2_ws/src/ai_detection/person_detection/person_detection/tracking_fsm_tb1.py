#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, Point

class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')

        # ================== Parameters (defaults baked-in) ==================
        self.declare_parameter('ns', 'tb1')
        self.declare_parameter('fsm_state_topic', 'fsm/state')
        self.declare_parameter('fsm_cmd_topic',   'fsm/cmd')
        self.declare_parameter('detection_topic', 'person_detected')
        self.declare_parameter('position_topic',  'person_position')  # Point(x=center_x, y=area)
        self.declare_parameter('cmd_vel_topic',   'cmd_vel')

        self.declare_parameter('tracking_state_name', 'TRACKING')
        self.declare_parameter('lost_timeout', 2.0)
        self.declare_parameter('image_width', 1920)  # 유지(미사용 가능)

        # 직진 속도/거리 제어 (회전 없음)
        self.declare_parameter('lin_base', 0.55)         # 기본 전진 속도
        self.declare_parameter('target_area', 240000.0)  # 더 가까운 목표
        self.declare_parameter('area_tolerance', 12000.0)

        # ================== Resolve ==================
        self.ns = self.get_parameter('ns').get_parameter_value().string_value.strip('/')
        def ns_join(suf: str) -> str:
            s = suf.lstrip('/')
            return f'/{self.ns}/{s}' if self.ns else f'/{s}'

        self.tracking_state_name = self.get_parameter('tracking_state_name').value.upper()
        self.fsm_state_topic = ns_join(self.get_parameter('fsm_state_topic').value)
        self.fsm_cmd_topic   = ns_join(self.get_parameter('fsm_cmd_topic').value)
        self.det_topic       = ns_join(self.get_parameter('detection_topic').value)
        self.pos_topic       = ns_join(self.get_parameter('position_topic').value)
        self.cmd_vel_topic   = ns_join(self.get_parameter('cmd_vel_topic').value)

        self.lost_timeout = float(self.get_parameter('lost_timeout').value)
        self.image_width  = int(self.get_parameter('image_width').value)

        self.lin_base     = float(self.get_parameter('lin_base').value)
        self.target_area  = float(self.get_parameter('target_area').value)
        self.area_tol     = float(self.get_parameter('area_tolerance').value)

        # ================== QoS ==================
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history     = HistoryPolicy.KEEP_LAST

        # ================== Pub/Sub ==================
        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_topic, qos)
        self.pub_fsm_cmd = self.create_publisher(String, self.fsm_cmd_topic, qos)
        self.sub_state = self.create_subscription(String, self.fsm_state_topic, self.cb_state, qos)
        self.sub_det   = self.create_subscription(Bool,   self.det_topic,       self.cb_detected, qos)
        self.sub_pos   = self.create_subscription(Point,  self.pos_topic,       self.cb_position, qos)

        # ================== Runtime ==================
        self.current_state = 'UNKNOWN'
        self.active = False
        self.person_detected = False
        self.center_x = None
        self.area = None
        self.last_seen_wall = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            "PersonTracker (no-rotation) started\n"
            f"- ns={self.ns or '(root)'}\n"
            f"- cmd_vel={self.cmd_vel_topic}\n"
            f"- target_area={self.target_area:.0f} tol=±{self.area_tol:.0f}\n"
            f"- lin_base={self.lin_base:.2f}, lost_timeout={self.lost_timeout}s"
        )

    # ================== Callbacks ==================
    def cb_state(self, msg: String):
        self.current_state = msg.data.strip().upper()
        self.get_logger().info(f"[FSM] state received: {self.current_state}")
        prev = self.active
        self.active = (self.current_state == self.tracking_state_name)
        if self.active and not prev:
            if self.person_detected:
                self.last_seen_wall = time.time()
            self.get_logger().info("[TRACK] Enter TRACKING mode.")
        elif not self.active and prev:
            self.stop_robot()
            self.get_logger().info("[TRACK] Leave TRACKING mode → stop.")

    def cb_detected(self, msg: Bool):
        self.person_detected = bool(msg.data)
        if self.person_detected:
            self.last_seen_wall = time.time()

    def cb_position(self, msg: Point):
        # x=center_x(미사용), y=area
        self.center_x = float(msg.x)
        self.area     = float(msg.y)
        self.last_seen_wall = time.time()

    # ================== Control loop (linear-only) ==================
    def control_loop(self):
        if not self.active:
            return

        now = time.time()
        if (not self.person_detected) or self.last_seen_wall == 0.0 or (now - self.last_seen_wall) > self.lost_timeout:
            self.stop_robot()
            self.publish_event('EV_TRACKING_DONE')
            self.get_logger().info(f"[TRACK] Lost target > {self.lost_timeout:.1f}s → EV_TRACKING_DONE.")
            return

        tw = Twist()

        if self.area is None:
            # 정보 없으면 천천히 전진
            tw.linear.x = self.lin_base * 0.5
            tw.angular.z = 0.0
            self.pub_cmd_vel.publish(tw)
            return

        low  = self.target_area - self.area_tol
        high = self.target_area + self.area_tol

        if self.area < low:
            # 멀다 → 전진 (회전 없음)
            tw.linear.x = self.lin_base
            tw.angular.z = 0.0
            self.get_logger().info(f'[TRACK] Approach (area={self.area:.0f} < {low:.0f}) → forward.')
        elif self.area > high:
            # 너무 가까움 → 후진 금지, 정지 유지
            tw.linear.x = 0.0
            tw.angular.z = 0.0
            self.get_logger().info(f'[TRACK] Hold-close (area={self.area:.0f} > {high:.0f}) → stop.')
        else:
            # 목표 거리 밴드 → 정지
            tw.linear.x = 0.0
            tw.angular.z = 0.0
            self.get_logger().info(f'[TRACK] Perfect spot (area≈{self.target_area:.0f}).')

        self.pub_cmd_vel.publish(tw)

    # ================== Helpers ==================
    def publish_event(self, ev_name: str):
        self.pub_fsm_cmd.publish(String(data=ev_name))

    def stop_robot(self):
        self.pub_cmd_vel.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = PersonTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

