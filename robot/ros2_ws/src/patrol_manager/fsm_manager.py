import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Empty

STATES = ["SETUP", "READY", "PATROLLING", "TRACKING", "RETURN", "PAUSED"]

# 이벤트 상수
EV_READY           = "EV_READY"            # SETUP  -> READY         (사람)
EV_START           = "EV_START"            # READY  -> PATROLLING    (백엔드)
EV_DETECTED        = "EV_DETECTED"         # PATROLLING -> TRACKING  (인식)
EV_TRACKING_DONE   = "EV_TRACKING_DONE"    # TRACKING   -> RETURN    (추적)
EV_RET             = "EV_RET"              # RETURN     -> SETUP     (추적)
EV_PAUSED          = "EV_PAUSED"           # *         -> PAUSED     (백엔드)
EV_RESUME          = "EV_RESUME"           # PAUSED    -> SETUP      (백엔드)

# 상태 전이 테이블 (현재상태 -> {이벤트: 다음상태})
TRANSITIONS = {
    "SETUP":       {EV_READY: "READY", EV_PAUSED: "PAUSED"},
    "READY":       {EV_START: "PATROLLING", EV_PAUSED: "PAUSED"},
    "PATROLLING":  {EV_DETECTED: "TRACKING", EV_PAUSED: "PAUSED"},
    "TRACKING":    {EV_TRACKING_DONE: "RETURN", EV_PAUSED: "PAUSED"},
    "RETURN":      {EV_RET: "SETUP", EV_PAUSED: "PAUSED"},
    "PAUSED":      {EV_RESUME: "SETUP"},
}

class RobotFSM:
    def __init__(self, node: Node, ns: str):
        self.node = node
        self.ns = ns
        self.state = "SETUP"
        
        qos_state = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # 라치드(최근값 유지)
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        # 상태 방송 퍼블리셔
        self.pub_state = node.create_publisher(String, f"/{ns}/fsm/state", 10)

        # 명령/이벤트 서브스크립션
        self.sub_cmd = node.create_subscription(String, f"/{ns}/fsm/cmd", self.on_cmd, 10)
        self.sub_detected = node.create_subscription(Empty, f"/{ns}/events/detected", self._wrap_empty(EV_DETECTED), 10)
        self.sub_tracking_done = node.create_subscription(Empty, f"/{ns}/events/tracking_done", self._wrap_empty(EV_TRACKING_DONE), 10)
        # EV_RET는 이름을 두 가지로 받도록 호환 (원하는 쪽만 써도 됨)
        self.sub_ret = node.create_subscription(Empty, f"/{ns}/events/ret", self._wrap_empty(EV_RET), 10)
        self.sub_return_done = node.create_subscription(Empty, f"/{ns}/events/return_done", self._wrap_empty(EV_RET), 10)

        # 초기 상태 브로드캐스트
        self.publish_state(initial=True)

    def _wrap_empty(self, ev_name):
        def _cb(_msg: Empty):
            self.on_event(ev_name)
        return _cb

    def on_cmd(self, msg: String):
        ev = msg.data.strip()
        self.on_event(ev)

    def on_event(self, ev: str):
        cur = self.state
        next_state = None
        # 공통 PAUSED 진입 처리(*->PAUSED)
        if ev == EV_PAUSED and cur != "PAUSED":
            next_state = "PAUSED"
        else:
            # 정상 전이 표에 따라 처리
            next_state = TRANSITIONS.get(cur, {}).get(ev)

        if next_state is None:
            self.node.get_logger().warn(f"[{self.ns}] Invalid transition")
            return

        if next_state == cur:
            return  # no-op

        self.state = next_state
        self.node.get_logger().info(f"[{self.ns}] {next_state}\t({cur} ---{ev}--> {next_state})")
        self.publish_state()

    def publish_state(self, initial=False):
        msg = String()
        msg.data = self.state
        self.pub_state.publish(msg)
        if initial:
            self.node.get_logger().info(f"[{self.ns}] {self.state}")

class FSMMgr(Node):
    def __init__(self, namespaces=("tb1", "tb2")):
        super().__init__("fsm_manager")
        self.fsms = {ns: RobotFSM(self, ns) for ns in namespaces}

def main():
    rclpy.init()
    node = FSMMgr(namespaces=("tb1", "tb2"))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
