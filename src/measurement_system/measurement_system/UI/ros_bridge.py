"""ROS2 UI bridge. Only sends an explicit StartMeasurement request."""
import json
import rclpy
from rclpy.node import Node
from PySide6.QtCore import QObject, QTimer, Signal
from measurement_system_interfaces.srv import StartMeasurement, StopMeasurement, MoveRobot, SaveMeasurement, GetRobotJoints, ReadLeica, SaveTablePoint, CreateMeasurement
from std_msgs.msg import Bool
from std_msgs.msg import String


class RosBridge(QObject):
    service_available = Signal(bool)
    start_result = Signal(bool, str)
    stop_result = Signal(bool, str)
    measurement_status = Signal(object)
    modules_status = Signal(object)
    calibration_result = Signal(bool, str)
    calibration_stop_result = Signal(bool, str)
    calibration_status = Signal(object)
    manual_move_result = Signal(bool, str)
    manual_leica_result = Signal(object)
    table_point_result = Signal(object)
    table_series_result = Signal(bool, int, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        self.node = Node("measurement_ui_bridge")
        self.client = self.node.create_client(StartMeasurement, "start_measurement")
        self.stop_client = self.node.create_client(StopMeasurement, "/stop_measurement")
        self._stop_future = None
        self._calib_start_future = None
        self._calib_stop_future = None
        self._calib_start_client = self.node.create_client(StartMeasurement, '/start_robot_calibration')
        self._calib_stop_client = self.node.create_client(StopMeasurement, '/stop_robot_calibration')
        self.node.create_subscription(String, '/robot_calibration/status', self._on_calibration_status, 10)
        self._robot_client = self.node.create_client(MoveRobot, "/move_robot")
        self._get_joints_client = self.node.create_client(GetRobotJoints, "/get_robot_joints")
        self._read_leica_client = self.node.create_client(ReadLeica, "/read_leica")
        self._manual_move_future = None
        self._manual_leica_future = None
        self._table_future = None
        self._table_series_future = None
        self._save_table_client = self.node.create_client(SaveTablePoint, "/save_table_point")
        self._create_table_series_client = self.node.create_client(CreateMeasurement, "/create_measurement")
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

    def _on_calibration_status(self, message):
        try:
            self.calibration_status.emit(json.loads(message.data))
        except ValueError:
            pass

    def _on_leica_health(self, message):
        self._leica_recent = bool(message.data)

    def check_modules(self):
        self.modules_status.emit({
            "robot": self._robot_client.service_is_ready(),
            "database": self._database_client.service_is_ready(),
            "controller": self.client.service_is_ready(),
            "leica": self._leica_recent,
            "calibration": self._calib_start_client.service_is_ready(),
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
        # Process manual move, Leica sample and table calibration replies on every spin.
        # Otherwise their futures remain pending forever and the UI never unlocks.
        self._consume_extra_futures()

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

        for attr, signal in (
            ('_calib_start_future', self.calibration_result),
            ('_calib_stop_future', self.calibration_stop_result),
        ):
            future = getattr(self, attr)
            if future is not None and future.done():
                setattr(self, attr, None)
                try:
                    reply = future.result()
                    signal.emit(bool(reply.success), str(reply.message))
                except Exception as exc:
                    signal.emit(False, str(exc))

    def manual_move(self, joints, velocity):
        if self._manual_move_future is not None or not self._robot_client.service_is_ready():
            raise RuntimeError("Robot service busy or unavailable")
        req = MoveRobot.Request()
        req.joints = [float(v) for v in joints]
        req.velocity = float(velocity)
        self.node.get_logger().info(
            f"Manual /move_robot request: joints_deg={list(req.joints)}, velocity={req.velocity}"
        )
        self._manual_move_future = self._robot_client.call_async(req)
        if self._manual_move_future is None:
            raise RuntimeError("ROS2 failed to create manual move future")

    def manual_measure(self):
        if self._manual_leica_future is not None or not self._read_leica_client.service_is_ready():
            raise RuntimeError("Leica service busy or unavailable")
        self._manual_leica_future = self._read_leica_client.call_async(ReadLeica.Request())

    def create_table_series(self, robot_name, measurement_name):
        if self._table_series_future is not None:
            raise RuntimeError("Creation of measurement series already running")
        if not self._create_table_series_client.service_is_ready():
            raise RuntimeError("/create_measurement unavailable")
        req = CreateMeasurement.Request()
        req.robot_name = str(robot_name).strip()
        req.measurement_name = str(measurement_name).strip()
        self._table_series_future = self._create_table_series_client.call_async(req)

    def table_measure(self, measurement_id, point_number):
        if self._table_future is not None:
            raise RuntimeError("Table point already being measured")
        if not self._save_table_client.service_is_ready():
            raise RuntimeError("/save_table_point unavailable")
        req = SaveTablePoint.Request()
        req.measurement_id = int(measurement_id)
        req.point_number = int(point_number)
        self._table_future = self._save_table_client.call_async(req)

    def _consume_extra_futures(self):
        if self._table_series_future is not None and self._table_series_future.done():
            future = self._table_series_future
            self._table_series_future = None
            try:
                reply = future.result()
                if reply is None:
                    raise RuntimeError("No create_measurement response")
                self.table_series_result.emit(bool(reply.success), int(reply.measurement_id), str(reply.message))
            except Exception as exc:
                self.table_series_result.emit(False, 0, str(exc))

        for attr, signal, mode in (
            ("_manual_move_future", self.manual_move_result, "move"),
            ("_manual_leica_future", self.manual_leica_result, "leica"),
            ("_table_future", self.table_point_result, "leica"),
        ):
            future = getattr(self, attr)
            if future is None or not future.done():
                continue
            setattr(self, attr, None)
            try:
                reply = future.result()
                if reply is None:
                    raise RuntimeError("No ROS2 service response")
                if mode == "move":
                    self.node.get_logger().info(
                        f"Manual /move_robot response: success={reply.success}, message={reply.message}"
                    )
                    signal.emit(bool(reply.success), str(reply.message))
                elif attr == "_table_future":
                    signal.emit({"success": bool(reply.success), "message": str(reply.message),
                                 "timestamp": int(reply.timestamp), "distance": float(reply.distance),
                                 "hz": float(reply.angle_hz), "vt": float(reply.angle_vt),
                                 "point_id": int(reply.point_id)})
                else:
                    signal.emit({"success": bool(reply.success), "message": str(reply.message),
                                 "timestamp": int(reply.timestamp), "distance": float(reply.distance),
                                 "hz": float(reply.angle_hz), "vt": float(reply.angle_vt)})
            except Exception as exc:
                if mode == "move":
                    signal.emit(False, str(exc))
                else:
                    signal.emit({"success": False, "message": str(exc)})

    def _check_service(self):
        self.service_available.emit(self.client.service_is_ready())
        self.check_modules()

    def start_measurement(self, csv_path, robot_name, measurement_name):
        if self._start_future is not None:
            raise RuntimeError("Čekám na odpověď předchozího požadavku")
        if not self.client.service_is_ready():
            raise RuntimeError("Služba /start_measurement není dostupná")
        req = StartMeasurement.Request()
        req.csv_path = csv_path
        req.robot_name = robot_name
        req.measurement_name = measurement_name
        self._start_future = self.client.call_async(req)

    def stop_measurement(self):
        if self._stop_future is not None:
            raise RuntimeError("Čeká se na předchozí STOP požadavek")
        if not self.stop_client.service_is_ready():
            raise RuntimeError("Služba /stop_measurement není dostupná")
        self._stop_future = self.stop_client.call_async(StopMeasurement.Request())

    def start_robot_calibration(self, csv_path, robot_name="KUKA LBR iiwa 14 R820", series_name="Kalibrace B"):
        if not self._calib_start_client.service_is_ready():
            raise RuntimeError('Není dostupná služba /start_robot_calibration')
        if getattr(self, '_calib_start_future', None) is not None:
            raise RuntimeError('Požadavek kalibrace se již zpracovává')
        request = StartMeasurement.Request()
        request.csv_path = csv_path
        request.robot_name = robot_name
        request.measurement_name = series_name
        self._calib_start_future = self._calib_start_client.call_async(request)

    def stop_robot_calibration(self):
        if self._calib_stop_future is not None:
            raise RuntimeError('Čeká se na potvrzení zastavení kalibrace')
        if not self._calib_stop_client.service_is_ready():
            raise RuntimeError('Není dostupná služba /stop_robot_calibration')
        self._calib_stop_future = self._calib_stop_client.call_async(StopMeasurement.Request())

    def close(self):
        self._spin_timer.stop()
        self._status_timer.stop()
        self.node.destroy_node()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()
