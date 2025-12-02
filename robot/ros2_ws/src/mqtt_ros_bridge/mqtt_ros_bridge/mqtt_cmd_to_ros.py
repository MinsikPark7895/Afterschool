import json, math, queue, os
from datetime import datetime, timezone
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import paho.mqtt.client as mqtt

HOST = os.getenv('MQTT_HOST', 'localhost')
PORT = int(os.getenv('MQTT_PORT', '1883'))
QOS  = 0

TOPICS = [
    ("to_robot/command/start_patrol", QOS),
    ("to_robot/command/move_to/+", QOS),
    ("to_robot/command/stop_patrol", QOS),
]

RID_ALIAS = {"robot_a": "tb1", "robot_b": "tb2", "tb1": "tb1", "tb2": "tb2"}

def now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")

def yaw_to_quat(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))

class MqttCmdToRos(Node):
    def __init__(self):
        super().__init__("mqtt_cmd_to_ros")
        self.q = queue.Queue()
        self.pubs_goal = {rid: self.create_publisher(PoseStamped, f"/{rid}/goal_pose", 10) for rid in ("tb1","tb2")}
        self.pubs_start = self.create_publisher(String,    "/to_robot/command/start_patrol", 10)
        self.pubs_stop = self.create_publisher(String,    "/to_robot/command/stop_patrol", 10)

        self.cli = mqtt.Client(client_id="mqtt_cmd_to_ros", clean_session=True)
        self.cli.on_message = self.on_message
        self.cli.connect(HOST, PORT, keepalive=30)
        for t, q in TOPICS:
            self.cli.subscribe(t, qos=q)
        self.cli.loop_start()

        self.timer = self.create_timer(0.05, self.process_queue)  # 20Hz

    def destroy_node(self):
        try:
            self.cli.loop_stop()
            self.cli.disconnect()
        finally:
            super().destroy_node()

    def on_message(self, _cli, _userdata, msg):
        try:
            j = json.loads(msg.payload.decode("utf-8"))
        except Exception as e:
            self.get_logger().warn(f"invalid JSON on {msg.topic}: {e}")
            return
        self.q.put((msg.topic, j))

    def process_queue(self):
        while not self.q.empty():
            topic, cmd = self.q.get_nowait()
            if topic == "to_robot/command/start_patrol":
                self.handle_start_patrol(cmd)
            elif topic.startswith("to_robot/command/move_to/"):
                rid_in_topic = topic.split("/", 3)[3]
                self.handle_move_to(rid_in_topic, cmd)
            elif topic == "to_robot/command/stop_patrol":
                self.handle_stop_patrol(cmd)

    def handle_start_patrol(self, cmd):
        payload = cmd.get("payload", {})
        data = payload.get("data", {})
        affected_robots = data.get("affected_robots", [])

        if payload.get("command_type") != "start_patrol":
            self.get_logger().warn("start_patrol invalid command_type"); return

        if not isinstance(affected_robots, list) or not affected_robots:
            self.get_logger().warn("start_patrol: affected_robots empty")
            return
         
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)

        self.pubs_start.publish(msg)
        self.get_logger().info(f"[CMD] start_patrol -> /command/patrol/start_patrol")
            
    def handle_move_to(self, rid_in_topic, cmd):
        header  = cmd.get("header", {})
        payload = cmd.get("payload", {})
        rid_raw = header.get("robot_id") or rid_in_topic
        rid = RID_ALIAS.get(rid_raw, rid_raw)
        if rid not in self.pubs_goal:
            self.get_logger().warn(f"move_to unknown rid={rid_raw}")
            return
        if payload.get("command_type") != "move_to":
            self.get_logger().warn("move_to invalid command_type"); return
        pos = payload.get("target_position", {})
        x, y, z = float(pos.get("x", 0.0)), float(pos.get("y", 0.0)), float(pos.get("z", 0.0))
        yaw = float(pos.get("yaw", 0.0)) if "yaw" in pos else 0.0
        qx, qy, qz, qw = yaw_to_quat(yaw)

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "map"
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        self.pubs_goal[rid].publish(ps)
        self.get_logger().info(f"[CMD] move_to -> /{rid}/goal_pose ({x:.2f},{y:.2f},{z:.2f}, yaw={yaw:.2f})")

    def handle_stop_patrol(self, cmd):
        payload = cmd.get("payload", {})
        data = payload.get("data", {})
        affected_robots = data.get("affected_robots", [])

        if payload.get("command_type") != "stop_patrol":
            self.get_logger().warn("stop_patrol invalid command_type"); return

        if not isinstance(affected_robots, list) or not affected_robots:
            self.get_logger().warn("stop_patrol: affected_robots empty")
            return
         
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pubs_stop.publish(msg)
        self.get_logger().info(f"[CMD] stop_patrol -> /command/patrol/stop_patrol")

def main():
    rclpy.init()
    node = MqttCmdToRos()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

