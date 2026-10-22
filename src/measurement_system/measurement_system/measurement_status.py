"""Small ROS2 status publisher independent of the UI."""
import json
from std_msgs.msg import String


class MeasurementStatusPublisher:
    def __init__(self, node):
        self.publisher = node.create_publisher(String, "/measurement/status", 10)

    def publish(self, state, completed, total, message=""):
        # One-based current point, zero if nothing is being measured.
        active = state in {"MOVING", "WAIT_MOVE", "SETTLING", "MEASURING", "WAIT_MEASUREMENT"}
        current_point = completed + 1 if active and completed < total else 0
        payload = {
            "state": state,
            "completed": int(completed),
            "total": int(total),
            "current_point": current_point,
            "message": str(message),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.publisher.publish(msg)