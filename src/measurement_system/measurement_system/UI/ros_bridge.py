"""ROS2 UI bridge. Only sends an explicit StartMeasurement request."""
import json
import rclpy
from rclpy.node import Node
from PySide6.QtCore import QObject, QTimer, Signal
from measurement_system_interfaces.srv import StartMeasurement
from std_msgs.msg import String
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError


class RosBridge(QObject):
    service_available = Signal(bool)
    start_result = Signal(bool, str)
    measurement_status = Signal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        self.node = Node("measurement_ui_bridge")
        self.client = self.node.create_client(StartMeasurement, "start_measurement")
        self._start_future = None
        self._status_subscription = self.node.create_subscription(
            String, "/measurement/status", self._on_status_message, 10
        )
        self._spin_timer = QTimer(self)
        self._spin_timer.timeout.connect(self._spin_once)
        self._spin_timer.start(20)
        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._check_service)
        self._status_timer.start(1000)
        self._check_service()

    def _on_status_message(self, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError("Status must be a JSON object")
            self.measurement_status.emit(data)
        except (ValueError, TypeError) as exc:
            self.node.get_logger().warning(f"Invalid measurement status: {exc}")

    def _spin_once(self):
        if not rclpy.ok():
            self.spin_timer.stop()
            return

        try:
            rclpy.spin_once(
                self.node,
                timeout_sec=0.0
            )

        except (RCLError, ExternalShutdownException):
            self.spin_timer.stop()

    def _check_service(self):
        self.service_available.emit(self.client.service_is_ready())

    def check_service(self):
        self._check_service()

    def start_measurement(self, csv_path):
        if self._start_future is not None:
            raise RuntimeError("Čekám na odpověď předchozího požadavku")
        if not self.client.service_is_ready():
            raise RuntimeError("Služba /start_measurement není dostupná")
        req = StartMeasurement.Request()
        req.csv_path = csv_path
        self._start_future = self.client.call_async(req)

    def close(self):
        self._spin_timer.stop()
        self._status_timer.stop()
        self.node.destroy_node()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()