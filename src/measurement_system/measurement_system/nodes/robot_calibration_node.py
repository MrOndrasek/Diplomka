"""ROS2 acquisition of paired robot targets and Leica raw measurements.

Does NOT compute the base transform; offline analysis is intentional.
"""
import json
import os
import time
import uuid
from enum import Enum, auto

from measurement_system.drivers.db_setup.databaze import get_connection
from measurement_system.drivers.db_setup.schema import ensure_measurement_tables
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from measurement_system_interfaces.srv import MoveRobot, StartMeasurement, StopMeasurement, GetRobotJoints
from measurement_system.measurement.position_importer import load_positions
from measurement_system.drivers.leika_driver.leika_ZMQ_driver import Leika_ZMQ_driver


class State(Enum):
    IDLE = auto()
    MOVING = auto()
    WAIT_MOVE = auto()
    SETTLING = auto()
    READING_JOINTS = auto()
    WAIT_JOINTS = auto()
    MEASURING = auto()
    FINISHED = auto()
    STOPPED = auto()
    ERROR = auto()


class RobotCalibrationNode(Node):
    def __init__(self):
        super().__init__('robot_calibration_node')
        self.declare_parameter('settle_seconds', 1.5)
        self.declare_parameter('velocity', 0.1)
        self.state = State.IDLE
        self.positions = []
        self.point_ids = []
        self.index = 0
        self.run_id = None
        self.future = None
        self.stop_requested = False
        self.settle_until = 0.0
        self.message = ''
        self.conn = None
        self.leica = None
        self.client = self.create_client(MoveRobot, '/move_robot')
        self.joints_client = self.create_client(GetRobotJoints, '/get_robot_joints')
        self.actual_joints = None
        self.start_srv = self.create_service(StartMeasurement, '/start_robot_calibration', self.on_start)
        self.stop_srv = self.create_service(StopMeasurement, '/stop_robot_calibration', self.on_stop)
        self.publisher = self.create_publisher(String, '/robot_calibration/status', 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.publish()

    def publish(self):
        message = String()
        message.data = json.dumps({
            'state': self.state.name,
            'completed': self.index,
            'total': len(self.positions),
            'current_point': self.point_ids[self.index] if self.index < len(self.point_ids) else '',
            'calibration_id': str(self.run_id) if self.run_id else '',
            'message': self.message,
        })
        self.publisher.publish(message)

    def on_start(self, request, response):
        if self.state not in (State.IDLE, State.FINISHED, State.STOPPED, State.ERROR):
            response.success = False
            response.message = 'Kalibrace již běží'
            return response
        try:
            positions, ids = load_positions(request.csv_path)
            if len(positions) < 3:
                raise ValueError('Pro kalibraci báze použij alespoň 3 různé polohy, doporučeno 6–10+')
            if not self.client.service_is_ready():
                raise RuntimeError('Služba /move_robot není dostupná')
            conn = get_connection()
            if conn is False or conn is None:
                raise RuntimeError("PostgreSQL connection failed for calibration B")
            ensure_measurement_tables(conn)
            calibration_id = uuid.uuid4()
            with conn.cursor() as cur:
                # Calibration uses its own series, independent from automatic measurement.
                robot_name = (request.robot_name or 'KUKA LBR iiwa 14 R820').strip()
                series_name = (request.measurement_name or 'Kalibrace B').strip()
                cur.execute("INSERT INTO robots(robot_name) VALUES (%s) RETURNING id", (robot_name,))
                robot_id = cur.fetchone()[0]
                cur.execute("INSERT INTO measurements(robot_id, name) VALUES (%s,%s) RETURNING id",
                            (robot_id, series_name))
                measurement_id = cur.fetchone()[0]
                cur.execute("""INSERT INTO robot_base_calibration_runs
                           (calibration_id,measurement_id) VALUES (%s,%s)""",
                            (str(calibration_id), measurement_id))
            conn.commit()
            leica = Leika_ZMQ_driver()
            leica.connection()
            # Owner of this SUB socket for the duration of calibration only.
            self.conn = conn
            self.leica = leica
            self.positions, self.point_ids = positions, ids
            self.index = 0
            self.run_id = calibration_id
            self.stop_requested = False
            self.message = 'Kalibrace zahájena'
            self.state = State.MOVING
            response.success = True
            response.message = f'Kalibrace spuštěna: {self.run_id}'
            self.publish()
        except Exception as exc:
            # On failed setup don't leave half-open connections.
            if 'leica' in locals():
                leica.end_connection()
            if 'conn' in locals():
                conn.close()
            self.conn = None
            self.leica = None
            response.success = False
            response.message = str(exc)
        return response

    def on_stop(self, request, response):
        running = self.state in (State.MOVING, State.WAIT_MOVE, State.SETTLING, State.MEASURING)
        self.stop_requested = running
        response.success = running
        response.message = ('Zastavení po aktuálním pohybu/měření požadováno' if running
                            else 'Kalibrace není aktivní')
        return response

    def finish(self, state, reason):
        self.state = state
        self.message = reason
        self.publish()
        self.get_logger().info(f'{state.name}: {reason}')
        self.stop_requested = False
        if self.leica is not None:
            self.leica.end_connection()
            self.leica = None
        if self.conn is not None:
            try:
                if self.run_id is not None:
                    with self.conn.cursor() as cur:
                        cur.execute("UPDATE robot_base_calibration_runs SET status=%s WHERE calibration_id=%s",
                                    (state.name, str(self.run_id)))
                    self.conn.commit()
            except Exception as exc:
                self.get_logger().error(f"Calibration status DB update failed: {exc}")
                self.conn.rollback()
            self.conn.close()
            self.conn = None

    def tick(self):
        if self.state in (State.IDLE, State.FINISHED, State.STOPPED, State.ERROR):
            return
        try:
            # Never issue a new robot move after STOP is requested.
            if self.stop_requested and self.state != State.WAIT_MOVE:
                self.finish(State.STOPPED, 'Zastaveno bez dalšího pohybu')
                return
            if self.state == State.MOVING:
                request = MoveRobot.Request()
                request.joints = [float(q) for q in self.positions[self.index]]
                request.velocity = float(self.get_parameter('velocity').value)
                self.future = self.client.call_async(request)
                self.state = State.WAIT_MOVE
                self.message = f'Najetí do bodu {self.point_ids[self.index]}'
                self.publish()
            elif self.state == State.WAIT_MOVE:
                if not self.future.done():
                    return
                result = self.future.result()
                if result is None or not result.success:
                    raise RuntimeError('Pohyb neúspěšný: ' + (result.message if result else 'bez odpovědi'))
                if self.stop_requested:
                    self.finish(State.STOPPED, 'Pohyb dokončen, další měření zrušeno')
                    return
                self.settle_until = time.monotonic() + float(self.get_parameter('settle_seconds').value)
                self.state = State.SETTLING
                self.message = 'Ustálení polohy'
                self.publish()
            elif self.state == State.SETTLING:
                if time.monotonic() >= self.settle_until:
                    self.state = State.READING_JOINTS
                    self.message = 'Načítám skutečné klouby KUKA'
                    self.publish()
            elif self.state == State.READING_JOINTS:
                if not self.joints_client.service_is_ready():
                    raise RuntimeError("GetRobotJoints is unavailable")
                self.future = self.joints_client.call_async(GetRobotJoints.Request())
                self.state = State.WAIT_JOINTS
            elif self.state == State.WAIT_JOINTS:
                if not self.future.done():
                    return
                reply = self.future.result()
                if reply is None or not reply.success or len(reply.joints) != 7:
                    raise RuntimeError("Cannot read measured joints for calibration")
                self.actual_joints = list(reply.joints)
                self.state = State.MEASURING
            elif self.state == State.MEASURING:
                # May wait up to 5 s. Dedicated node; no GUI blocking.
                reading = self.leica.measure_point()
                if not isinstance(reading, (tuple, list)) or len(reading) != 4:
                    raise RuntimeError('Leica nevrátila platné měření')
                timestamp, distance, hz, vt = reading
                with self.conn.cursor() as cur:
                    cur.execute('''INSERT INTO robot_base_calibration_points
                      (calibration_id, point_id, measured_at, distance, angle_hz, angle_vt,
                       q1,q2,q3,q4,q5,q6,q7)
                      VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)''',
                      (str(self.run_id), str(self.point_ids[self.index]), int(timestamp),
                       float(distance), float(hz), float(vt),
                       *[float(q) for q in self.actual_joints]))
                self.conn.commit()
                self.index += 1
                self.publish()
                if self.stop_requested:
                    self.finish(State.STOPPED, 'Poslední měření uloženo, sekvence zastavena')
                elif self.index >= len(self.positions):
                    self.finish(State.FINISHED, f'Kalibrace {self.run_id} uložena')
                else:
                    self.state = State.MOVING
        except Exception as exc:
            if self.conn is not None:
                self.conn.rollback()
            self.finish(State.ERROR, str(exc))

    def close(self):
        if self.leica is not None:
            self.leica.end_connection()
        if self.conn is not None:
            self.conn.close()


def main(args=None):
    rclpy.init(args=args)
    node = RobotCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
