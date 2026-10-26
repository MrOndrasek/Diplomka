"""ROS2 UI bridge. Only sends an explicit StartMeasurement request."""
import json
import rclpy
from rclpy.node import Node
from PySide6.QtCore import QObject, QTimer, Signal
from measurement_system_interfaces.srv import StartMeasurement, StopMeasurement, MoveRobot, SaveMeasurement
from std_msgs.msg import Bool
from std_msgs.msg import String


class RosBridge(QObject):
    service_available = Signal(bool)
    start_result = Signal(bool, str)
    stop_result = Signal(bool, str)
    measurement_status = Signal(object)
    modules_status = Signal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        self.node = Node("measurement_ui_bridge")
        self.client = self.node.create_client(StartMeasurement, "start_measurement")
        self.stop_client = self.node.create_client(StopMeasurement, "/stop_measurement")
        self._stop_future = None
        self._robot_client = self.node.create_client(MoveRobot, "/move_robot")
        self._database_client = self.node.create_client(SaveMeasurement, "/save_measurement")
        self._leica_recent = False
        self.node.create_subscription(Bool, "/leica/data_available", self._on_leica_health, 10)
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

    def _on_leica_health(self, message):
        self._leica_recent = bool(message.data)

    def check_modules(self):
        self.modules_status.emit({
            "robot": self._robot_client.service_is_ready(),
            "database": self._database_client.service_is_ready(),
            "controller": self.client.service_is_ready(),
            "leica": self._leica_recent,
        })

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
            self._spin_timer.stop()
            self._status_timer.stop()
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception as exc:
            self._spin_timer.stop()
            self._status_timer.stop()
            self.node.get_logger().error(f"ROS2 spin ukončen: {exc}")
            return
        if self._start_future is not None and self._start_future.done():
            future = self._start_future
            self._start_future = None
            try:
                reply = future.result()
                if reply is None:
                    raise RuntimeError("Služba nevrátila odpověď")
                self.start_result.emit(bool(reply.success), str(reply.message))
            except Exception as exc:
                self.start_result.emit(False, str(exc))

        if self._stop_future is not None and self._stop_future.done():
            future = self._stop_future
            self._stop_future = None
            try:
                reply = future.result()
                if reply is None:
                    raise RuntimeError("Služba nevrátila odpověď")
                self.stop_result.emit(bool(reply.success), str(reply.message))
            except Exception as exc:
                self.stop_result.emit(False, str(exc))

    def _check_service(self):
        self.service_available.emit(self.client.service_is_ready())
        self.check_modules()

    def start_measurement(self, csv_path):
        if self._start_future is not None:
            raise RuntimeError("Čekám na odpověď předchozího požadavku")
        if not self.client.service_is_ready():
            raise RuntimeError("Služba /start_measurement není dostupná")
        req = StartMeasurement.Request()
        req.csv_path = csv_path
        self._start_future = self.client.call_async(req)

    def stop_measurement(self):
        if self._stop_future is not None:
            raise RuntimeError("Čeká se na předchozí STOP požadavek")
        if not self.stop_client.service_is_ready():
            raise RuntimeError("Služba /stop_measurement není dostupná")
        self._stop_future = self.stop_client.call_async(StopMeasurement.Request())

    def close(self):
        self._spin_timer.stop()
        self._status_timer.stop()
        self.node.destroy_node()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()