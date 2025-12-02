#!/usr/bin/env python3
import json
from typing import List, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Empty

# --------------------
# States / Events
# --------------------
STATES = ["SETUP", "READY", "PATROLLING", "TRACKING", "RETURN"]

EV_READY         = "EV_READY"          # SETUP  -> READY        (사람/터미널)
EV_START         = "EV_START"          # READY  -> PATROLLING   (백엔드)
EV_DETECTED      = "EV_DETECTED"       # PATROL -> TRACKING     (인식)
EV_TRACKING_DONE = "EV_TRACKING_DONE"  # TRACK  -> RETURN       (추적)
EV_RET           = "EV_RET"            # RETURN -> READY        (주행)
EV_STOP          = "EV_STOP"           # *      -> RETURN       (백엔드)

# 상태 전이 테이블 (현재상태 -> {이벤트: 다음상태})
TRANSITIONS: Dict[str, Dict[str, str]] = {
    "SETUP":       {EV_READY: "READY"},
    "READY":       {EV_START: "PATROLLING"},
    "PATROLLING":  {EV_DETECTED: "TRACKING"},
    "TRACKING":    {EV_TRACKING_DONE: "RETURN"},
    "RETURN":      {EV_RET: "READY"},
}

# 로봇 이름(ID) -> 네임스페이스(NS) 매핑 (참고용)
ROBOT_ID_TO_NS: Dict[str, str] = {
    "robot_a": "tb1",
    "robot_b": "tb2",
}

class RobotFSM:
    def __init__(self, node: Node, ns: str):
        self.node = node
        self.ns = ns
        self.state = "SETUP"

        # 상태 방송 퍼블리셔 (라치드 QoS)
        self.qos_state = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.pub_state = node.create_publisher(String, f"/{ns}/fsm/state", self.qos_state)

        # 명령/이벤트 구독
        self.sub_cmd = node.create_subscription(String, f"/{ns}/fsm/cmd", self.on_cmd, 10)
        self.sub_detected = node.create_subscription(Empty, f"/{ns}/events/detected", self._wrap_empty(EV_DETECTED), 10)
        self.sub_tracking_done = node.create_subscription(Empty, f"/{ns}/events/tracking_done", self._wrap_empty(EV_TRACKING_DONE), 10)
        # EV_RET 호환 이벤트명 둘 다 허용
        self.sub_ret = node.create_subscription(Empty, f"/{ns}/events/ret", self._wrap_empty(EV_RET), 10)
        self.sub_return_done = node.create_subscription(Empty, f"/{ns}/events/return_done", self._wrap_empty(EV_RET), 10)

        # 초기 상태 브로드캐스트
        self.publish_state(initial=True)

    def _wrap_empty(self, ev_name: str):
        def _cb(_msg: Empty):
            self.on_event(ev_name)
        return _cb

    def on_cmd(self, msg: String):
        ev = msg.data.strip()
        self.on_event(ev)

    def on_event(self, ev: str):
        cur = self.state

        # 1) EV_STOP: 전역 전이 (* -> RETURN)
        if ev == EV_STOP:
            next_state = "RETURN"
        else:
            # 2) 테이블 기반 전이
            next_state = TRANSITIONS.get(cur, {}).get(ev)

        if next_state is None:
            self.node.get_logger().warn(f"[{self.ns}] Invalid transition: {cur} ---{ev}--> ?")
            return

        # self-loop 허용 (특히 RETURN에서 EV_STOP 등)
        if next_state == cur:
            self.node.get_logger().info(f"[{self.ns}] Stay {cur}\t({cur} ---{ev}--> {next_state})")
            # 필요 시 상태 재방송:
            # self.publish_state()
            return

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
        self.fsms: Dict[str, RobotFSM] = {ns: RobotFSM(self, ns) for ns in namespaces}

        # 백엔드 명령 구독: start_patrol -> READY인 FSM만 EV_START
        self.sub_start_patrol = self.create_subscription(
            String,
            "to_robot/command/start_patrol",
            self.on_start_patrol_command,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        # 백엔드 명령 구독: stop_patrol -> 모든 FSM에 EV_STOP
        self.sub_stop_patrol = self.create_subscription(
            String,
            "to_robot/command/stop_patrol",
            self.on_stop_patrol_command,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

    # -------------------------------
    # 외부 명령 처리: start_patrol
    # - 메시지가 오면 "READY 상태인 모든 로봇"만 PATROLLING으로 전환
    # - READY가 아니면 무효(스킵)
    # - affected_robots/response_required/timeout 은 동작에 영향 X (로그만)
    # -------------------------------
    def on_start_patrol_command(self, msg: String):
        raw = msg.data
        
        # raw를 dict로
        try:
            data = json.loads(raw)
        except Exception as e:
            self.get_logger().error(f"[start_patrol] JSON parse error: {e} | raw={raw}")
            return
        
        if data.get("command_type", "") != "start_patrol":
            self.get_logger().warn(f"[start_patrol] Unsupported command_type: {payload.get('command_type')}")
            return

        started: List[str] = []
        skipped: List[str] = []
        for ns, fsm in self.fsms.items():
            if fsm.state == "READY":
                fsm.on_event(EV_START)  # READY -> PATROLLING
                started.append(ns)
            else:
                skipped.append(f"{ns}:{fsm.state}")

    # -------------------------------
    # 외부 명령 처리: stop_patrol
    # - 모든 FSM에 EV_STOP 적용 (* -> RETURN)
    # -------------------------------
    def on_stop_patrol_command(self, msg: String):
        raw = msg.data
        try:
            data = json.loads(raw)
        except Exception as e:
            self.get_logger().error(f"[stop_patrol] JSON parse error: {e} | raw={raw}")
            return

        if data.get("command_type", "") != "stop_patrol":
            self.get_logger().warn(f"[stop_patrol] Unsupported command_type: {payload.get('command_type')}")
            return

        for ns, fsm in self.fsms.items():
            fsm.on_event(EV_STOP)

def main():
    rclpy.init()
    node = FSMMgr(namespaces=tuple(set(ROBOT_ID_TO_NS.values())))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

