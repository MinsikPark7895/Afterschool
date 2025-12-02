import json, time, math
from datetime import datetime, timezone, timedelta

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from std_msgs.msg import Bool, String
import paho.mqtt.client as mqtt

ROBOT_IDS = ("tb1", "tb2")
ROBOT_ZONES = {"tb1": "zone_a", "tb2": "zone_b"}

TOPIC_FMT_BASIC         = "from_robot/status/basic/{robot_id}"
TOPIC_FMT_DETAIL        = "from_robot/status/detail/{robot_id}"
TOPIC_FMT_EVENT_PERSON  = "from_robot/event/detection/{robot_id}"
TOPIC_FMT_EVENT_DONE    = "from_robot/event/mission_done/{robot_id}"
TOPIC_SYSTEM_EVENT      = "from_robot/event/system_status"

def now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")

class RobotCtx:
    def __init__(self, rid: str):
        self.rid = rid
        self.zone = ROBOT_ZONES[rid]
        self.pose = None
        self.goal = None
        self.speed = 0.0
        self.state = "PATROLLING"
        self.battery = 89.5 if rid == "tb1" else 97.5
        self.last_detail_payload_str = None

class MqttRosBridge(Node):
    def __init__(self):
        super().__init__("mqtt_ros_bridge")

        self.declare_parameter('mqtt_host', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('qos', 0)
        self.mqtt_host = self.get_parameter('mqtt_host').value
        self.mqtt_port = int(self.get_parameter('mqtt_port').value)
        self.qos = int(self.get_parameter('qos').value)

        self.ctx = {rid: RobotCtx(rid) for rid in ROBOT_IDS}

        for rid in ROBOT_IDS:
            self.create_subscription(PoseWithCovarianceStamped, f"/{rid}/amcl_pose", self._mk_on_pose(rid), 10)
            self.create_subscription(Twist, f"/{rid}/cmd_vel", self._mk_on_twist(rid), 10)
            self.create_subscription(PoseStamped, f"/{rid}/goal_pose", self._mk_on_goal(rid), 10)
            self.create_subscription(Bool, f"/{rid}/event/detection", self._mk_on_detect(rid), 10)
            self.create_subscription(Bool, f"/{rid}/event/mission_done", self._mk_on_mission_done(rid), 10)
            self.create_subscription(String, f"/{rid}/fsm/state", self._mk_on_state(rid), 10)

        self.cli = mqtt.Client(client_id="ros2_mqtt_status_pub", clean_session=True)
        self.cli.connect(self.mqtt_host, self.mqtt_port, keepalive=30)
        self.cli.loop_start()

        self.timer = self.create_timer(1.0, self.tick_1hz)

    def destroy_node(self):
        try:
            time.sleep(0.2)
            self.cli.loop_stop()
            self.cli.disconnect()
        finally:
            super().destroy_node()

    def _mk_on_pose(self, rid):
        def cb(msg: PoseWithCovarianceStamped):
            self.ctx[rid].pose = msg.pose.pose
        return cb

    def _mk_on_twist(self, rid):
        def cb(msg: Twist):
            v = msg.linear
            self.ctx[rid].speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
        return cb

    def _mk_on_goal(self, rid):
        def cb(msg: PoseStamped):
            self.ctx[rid].goal = msg.pose
        return cb

    def _mk_on_detect(self, rid):
        def cb(msg: Bool):
            if msg.data:
                self.publish_event_detection(rid)
        return cb

    def _mk_on_mission_done(self, rid):
        def cb(msg: Bool):
            if msg.data:
                self.publish_event_mission_done(rid)
        return cb

    def _mk_on_state(self, rid):
        def cb(msg: String):
            self.ctx[rid].state = msg.data if msg.data else self.ctx[rid].state
        return cb

    def publish_json(self, topic, payload):
        s = json.dumps(payload, ensure_ascii=False)
        info = self.cli.publish(topic, s, qos=self.qos, retain=False)
        info.wait_for_publish()
        self.get_logger().info(f"[PUB] {topic}")

    def build_basic_status(self, rid):
        c = self.ctx[rid]
        pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        if c.pose:
            pos["x"] = c.pose.position.x
            pos["y"] = c.pose.position.y
            pos["z"] = c.pose.position.z
        return {
            "header": {"timestamp": now_iso(), "message_id": "msg_001", "robot_id": rid},
            "payload": {
                "position": pos,
                "state": c.state,
                "battery_level": round(c.battery, 2),
                "current_zone": c.zone,
                "current_mission": {"type": "patrol", "status": "in_progress", "progress": 65}
            }
        }

    def build_detail_status(self, rid):
        c = self.ctx[rid]
        tgt = {"x": 0.0, "y": 0.0}
        if c.goal:
            tgt["x"] = c.goal.position.x
            tgt["y"] = c.goal.position.y
        payload = {
            "health": {
                "overall_status": "healthy",
                "hardware": {"lidar": "active", "camera": "active", "imu": "active", "wheels": "active", "motors": "active"},
                "software": {"slam_node": "running", "navigation_node": "running", "detection_node": "running", "patrol_manager": "running", "mqtt_client": "connected"}
            },
            "charging_status": "discharging",
            "target_position": tgt,
            "assigned_zone": c.zone,
            "speed": round(c.speed, 3),
            "recent_errors": [{
                "timestamp": "2025-09-22T14:29:45.000Z",
                "level": "WARNING", "component": "camera", "message": "Frame rate dropped below 25 FPS"
            }]
        }
        return {"header": {"timestamp": now_iso(), "message_id": "msg_002", "robot_id": rid}, "payload": payload}

    def publish_event_detection(self, rid):
        c = self.ctx[rid]
        loc = {"x": 0.0, "y": 0.0, "z": 0.0, "zone": c.zone}
        if c.pose:
            loc["x"] = c.pose.position.x
            loc["y"] = c.pose.position.y
            loc["z"] = c.pose.position.z
        payload = {
            "header": {"timestamp": now_iso(), "message_id": "msg_003", "robot_id": rid},
            "payload": {
                "event_type": "person_detected",
                "severity": "critical",
                "detection_info": {
                    "confidence": 0.94, "person_count": 1,
                    "bounding_boxes": [{"x": 320, "y": 240, "width": 80, "height": 120, "confidence": 0.94}]
                },
                "location": loc,
                "threat_assessment": {"is_intruder": True, "assessment_reason": "detected after hours in restricted area"},
                "evidence": {"image_path": f"/evidence/detection_{rid}_20250922_142530.jpg"}
            }
        }
        self.publish_json(TOPIC_FMT_EVENT_PERSON.format(robot_id=rid), payload)

    def publish_event_mission_done(self, rid):
        end_time = datetime.now(timezone.utc)
        start_time = end_time - timedelta(minutes=30)
        payload = {
            "header": {"timestamp": now_iso(), "message_id": "msg_004", "robot_id": rid},
            "payload": {
                "event_type": "patrol_completed",
                "mission_type": "patrol",
                "zone": ROBOT_ZONES[rid],
                "mission_summary": {
                    "start_time": start_time.isoformat(timespec="milliseconds").replace("+00:00", "Z"),
                    "end_time": end_time.isoformat(timespec="milliseconds").replace("+00:00", "Z"),
                    "duration": int((end_time - start_time).total_seconds()),
                    "success": True
                },
                "patrol_details": {"waypoints_visited": 4, "total_distance": 25.3, "detections_made": 0, "issues_encountered": 0},
                "next_action": "STANDBY"
            }
        }
        self.publish_json(TOPIC_FMT_EVENT_DONE.format(robot_id=rid), payload)

    def tick_1hz(self):
        for rid in ROBOT_IDS:
            self.publish_json(TOPIC_FMT_BASIC.format(robot_id=rid), self.build_basic_status(rid))
        for rid in ROBOT_IDS:
            detail = self.build_detail_status(rid)
            s = json.dumps(detail["payload"], sort_keys=True)
            if self.ctx[rid].last_detail_payload_str != s:
                self.publish_json(TOPIC_FMT_DETAIL.format(robot_id=rid), detail)
                self.ctx[rid].last_detail_payload_str = s

def main():
    rclpy.init()
    node = MqttRosBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
