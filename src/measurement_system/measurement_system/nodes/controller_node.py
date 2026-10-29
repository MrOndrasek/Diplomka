#!/usr/bin/env python3
"""ROS2 measurement state machine; STOP means finish current operation."""
import time
from enum import Enum, auto
import rclpy
from rclpy.node import Node
from measurement_system_interfaces.srv import MoveRobot, SaveMeasurement, StartMeasurement, StopMeasurement, GetRobotJoints, CreateMeasurement
from measurement_system.measurement.position_importer import load_positions
from measurement_system.measurement_status import MeasurementStatusPublisher


class State(Enum):
    IDLE = auto()
    INITIALIZING_DB = auto()
    WAIT_DB_READY = auto()
    WAIT_SERIES = auto()
    MOVING = auto()
    WAIT_MOVE = auto()
    SETTLING = auto()
    READING_JOINTS = auto()
    WAIT_JOINTS = auto()
    MEASURING = auto()
    WAIT_MEASUREMENT = auto()
    FINISHED = auto()
    STOPPED = auto()
    ERROR = auto()


class MeasurementController(Node):
    def __init__(self):
        super().__init__("measurement_controller")
        self.state = State.IDLE
        self.status_publisher = MeasurementStatusPublisher(self)
        self.positions = []
        self.point_ids = []
        self.position_index = 0
        self.measurement_id = 0
        self.actual_joints = None
        self.future = None
        self.settle_until = 0.0
        self.wait_since = None
        self.stop_requested = False
        self.restart_blocked = False
        self.declare_parameter("velocity", 0.1)
        self.declare_parameter("settle_seconds", 1.0)
        self.declare_parameter("operation_timeout_seconds", 120.0)
        self.start_service = self.create_service(StartMeasurement, "/start_measurement", self.callback_start_measurement)
        self.stop_service = self.create_service(StopMeasurement, "/stop_measurement", self.callback_stop_measurement)
        self.robot_client = self.create_client(MoveRobot, "/move_robot")
        self.measurement_client = self.create_client(SaveMeasurement, "/save_measurement")
        self.joints_client = self.create_client(GetRobotJoints, "/get_robot_joints")
        self.series_client = self.create_client(CreateMeasurement, "/create_measurement")
        self.timer = self.create_timer(0.1, self.update)
        self.publish()

    def publish(self, message=""):
        self.status_publisher.publish(self.state.name, self.position_index, len(self.positions), message)

    def callback_start_measurement(self, request, response):
        if self.state not in (State.IDLE, State.FINISHED, State.STOPPED, State.ERROR):
            response.success = False
            response.message = "Measurement already active"
            return response
        if self.restart_blocked:
            response.success = False
            response.message = "Previous operation timed out; restart controller only after verifying robot stopped"
            return response
        try:
            positions, ids = load_positions(request.csv_path)
            if not (self.robot_client.service_is_ready() and self.measurement_client.service_is_ready()
                    and self.joints_client.service_is_ready() and self.series_client.service_is_ready()):
                raise RuntimeError("Robot or database measurement service unavailable")
            if not request.robot_name.strip() or not request.measurement_name.strip():
                raise ValueError("Select robot and measurement series name")
            self.robot_name = request.robot_name.strip()
            self.measurement_name = request.measurement_name.strip()
            self.measurement_id = 0
            self.actual_joints = None
            self.positions, self.point_ids = positions, ids
            self.position_index = 0
            self.stop_requested = False
            self.future = None
            self.wait_since = None
            self.state = State.INITIALIZING_DB
            self.publish("Sequence accepted; preparing PostgreSQL tables")
            response.success = True
            response.message = f"Sequence accepted: {len(positions)} targets"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def callback_stop_measurement(self, request, response):
        if self.state in (State.IDLE, State.FINISHED, State.STOPPED, State.ERROR):
            response.success = False
            response.message = "No active sequence"
        else:
            self.stop_requested = True
            response.success = True
            response.message = "Stop accepted; current movement/measurement will finish"
        return response

    def finish(self, state, message):
        self.state = state
        self.stop_requested = False
        self.future = None
        self.wait_since = None
        self.publish(message)
        self.get_logger().info(f"{state.name}: {message}")

    def fail(self, message):
        self.finish(State.ERROR, message)
        self.get_logger().error(message)

    def update(self):
        if self.state in (State.IDLE, State.FINISHED, State.STOPPED, State.ERROR):
            return
        # Never start another operation after STOP has been requested.
        if self.stop_requested and self.state not in (State.WAIT_MOVE, State.WAIT_MEASUREMENT):
            self.finish(State.STOPPED, "Stopped before next operation")
            return
        try:
            if self.state == State.INITIALIZING_DB:
                if not self.measurement_client.service_is_ready():
                    raise RuntimeError("Database measurement service unavailable")
                request = SaveMeasurement.Request()
                request.save = False  # non-measuring preflight, initializes schema
                request.joints = []
                self.future = self.measurement_client.call_async(request)
                self.wait_since = time.monotonic()
                self.state = State.WAIT_DB_READY
                self.publish("Preparing PostgreSQL tables")

            elif self.state == State.WAIT_DB_READY:
                if not self.future.done():
                    self.check_timeout("Database initialization")
                    return
                reply = self.future.result()
                if reply is None or not reply.success:
                    raise RuntimeError("Database initialization failed: " + (reply.message if reply else "no response"))
                self.get_logger().info(f"Database preflight OK: {reply.message}")
                if self.stop_requested:
                    self.finish(State.STOPPED, "Stopped before robot movement")
                else:
                    req = CreateMeasurement.Request()
                    req.robot_name = self.robot_name
                    req.measurement_name = self.measurement_name
                    self.future = self.series_client.call_async(req)
                    self.wait_since = time.monotonic()
                    self.state = State.WAIT_SERIES
                    self.publish("Creating measurement series")

            elif self.state == State.WAIT_SERIES:
                if not self.future.done():
                    self.check_timeout("Create measurement series")
                    return
                reply = self.future.result()
                if reply is None or not reply.success or reply.measurement_id <= 0:
                    raise RuntimeError("Cannot create measurement series: " + (reply.message if reply else "no reply"))
                self.measurement_id = int(reply.measurement_id)
                if self.stop_requested:
                    self.finish(State.STOPPED, "Stopped before robot movement")
                else:
                    self.state = State.MOVING
                    self.publish(f"Series ID {self.measurement_id}; moving")

            elif self.state == State.MOVING:
                if not self.robot_client.service_is_ready():
                    raise RuntimeError("Robot service unavailable")
                request = MoveRobot.Request()
                request.joints = [float(q) for q in self.positions[self.position_index]]
                request.velocity = float(self.get_parameter("velocity").value)
                self.future = self.robot_client.call_async(request)
                self.wait_since = time.monotonic()
                self.state = State.WAIT_MOVE
                self.publish("Moving")

            elif self.state == State.WAIT_MOVE:
                if not self.future.done():
                    self.check_timeout("Robot movement")
                    return
                reply = self.future.result()
                if reply is None or not reply.success:
                    raise RuntimeError("Movement failed: " + (reply.message if reply else "no response"))
                if self.stop_requested:
                    self.finish(State.STOPPED, "Current movement completed; measurement skipped")
                    return
                self.settle_until = time.monotonic() + float(self.get_parameter("settle_seconds").value)
                self.state = State.SETTLING
                self.publish("Settling")

            elif self.state == State.SETTLING:
                if time.monotonic() >= self.settle_until:
                    self.state = State.READING_JOINTS
                    self.publish("Reading real KUKA joint angles")

            elif self.state == State.READING_JOINTS:
                if not self.joints_client.service_is_ready():
                    raise RuntimeError("Robot joint readout service unavailable")
                self.future = self.joints_client.call_async(GetRobotJoints.Request())
                self.wait_since = time.monotonic()
                self.state = State.WAIT_JOINTS

            elif self.state == State.WAIT_JOINTS:
                if not self.future.done():
                    self.check_timeout("Robot joint readout")
                    return
                reply = self.future.result()
                if reply is None or not reply.success or len(reply.joints) != 7:
                    raise RuntimeError("Robot joint readout failed: " + (reply.message if reply else "no response"))
                self.actual_joints = [float(q) for q in reply.joints]
                self.state = State.MEASURING
                self.publish("Real joints acquired; requesting Leica sample")

            elif self.state == State.MEASURING:
                if not self.measurement_client.service_is_ready():
                    raise RuntimeError("Measurement service unavailable")
                request = SaveMeasurement.Request()
                request.save = True
                if self.actual_joints is None or len(self.actual_joints) != 7:
                    raise RuntimeError("No measured KUKA joint positions")
                request.joints = self.actual_joints
                request.measurement_id = self.measurement_id
                self.future = self.measurement_client.call_async(request)
                self.wait_since = time.monotonic()
                self.state = State.WAIT_MEASUREMENT
                self.publish("Waiting for Leica sample and DB commit")

            elif self.state == State.WAIT_MEASUREMENT:
                if not self.future.done():
                    self.check_timeout("Measurement")
                    return
                reply = self.future.result()
                if reply is None or not reply.success:
                    raise RuntimeError("Measurement failed: " + (reply.message if reply else "no response"))
                self.position_index += 1
                self.actual_joints = None
                if self.stop_requested:
                    self.finish(State.STOPPED, "Last measurement committed; sequence stopped")
                elif self.position_index >= len(self.positions):
                    self.finish(State.FINISHED, "All measurements committed")
                else:
                    self.state = State.MOVING
                    self.publish("Next target")
        except Exception as exc:
            self.fail(str(exc))

    def check_timeout(self, operation):
        limit = float(self.get_parameter("operation_timeout_seconds").value)
        if self.wait_since is not None and time.monotonic() - self.wait_since > limit:
            # The external service may still be processing; never claim robot stopped.
            self.restart_blocked = True
            self.fail(f"{operation} response timed out; verify robot state before restarting")


def main(args=None):
    rclpy.init(args=args)
    node = MeasurementController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
