#!/usr/bin/env python3
"""ROS2 endpoint for one Leica sample and an atomic PostgreSQL insert."""
import rclpy
from rclpy.node import Node
from measurement_system_interfaces.srv import SaveMeasurement, ReadLeica, CreateMeasurement, SaveTablePoint
from measurement_system.drivers.db_setup.databaze import get_connection as existing_db_connection
from measurement_system.drivers.db_setup.schema import ensure_measurement_tables
from measurement_system.drivers.leika_driver.leika_ZMQ_driver import Leika_ZMQ_driver


class DatabaseNode(Node):
    def __init__(self):
        super().__init__("database_node")
        self.conn = self.get_connection()
        if self.conn is None:
            raise RuntimeError("PostgreSQL unavailable; database_node will not start. Open Settings in the UI.")
        self.leika = Leika_ZMQ_driver()
        self.leika.connection()
        from measurement_system.connection_profiles import leica_endpoint
        self.get_logger().info(f"Leica subscriber initialized on {leica_endpoint()}")
        self.read_service_ = self.create_service(ReadLeica, "/read_leica", self.callback_read_leica)
        self.create_measurement_service_ = self.create_service(CreateMeasurement, "/create_measurement", self.callback_create_measurement)
        self.table_save_service_ = self.create_service(
            SaveTablePoint, "/save_table_point", self.callback_save_table_point
        )
        self.save_service_ = self.create_service(
            SaveMeasurement, "/save_measurement", self.callback_save_measurement
        )

    def get_connection(self):
        # Stejná konfigurace jako funkční db_setup/databaze.py.
        # V budoucnu přesunout credentials do ~/.pgpass a sjednotit nastavení.
        conn = existing_db_connection()
        if conn is False or conn is None:
            self.get_logger().error("PostgreSQL connection failed (db_setup/databaze.py)")
            return None
        try:
            with conn.cursor() as cur:
                cur.execute("SELECT 1")
                cur.fetchone()
            conn.rollback()  # ukončení transakce pouze pro health-check
            self.get_logger().info("PostgreSQL SELECT 1 OK")
            return conn
        except Exception as exc:
            self.get_logger().error(f"PostgreSQL health-check failed: {exc}")
            conn.close()
            return None

    def callback_create_measurement(self, request, response):
        try:
            if self.conn is None or self.conn.closed:
                raise RuntimeError("Database is disconnected")
            if not request.robot_name.strip() or not request.measurement_name.strip():
                raise ValueError("Robot and measurement name are mandatory")
            ensure_measurement_tables(self.conn)
            with self.conn.cursor() as cur:
                cur.execute("SELECT id FROM robots WHERE robot_name=%s ORDER BY id LIMIT 1", (request.robot_name.strip(),))
                row = cur.fetchone()
                if row is None:
                    cur.execute("INSERT INTO robots(robot_name) VALUES (%s) RETURNING id", (request.robot_name.strip(),))
                    row = cur.fetchone()
                cur.execute("INSERT INTO measurements(robot_id,name) VALUES (%s,%s) RETURNING id",
                            (row[0], request.measurement_name.strip()))
                response.measurement_id = int(cur.fetchone()[0])
            self.conn.commit()
            response.success = True
            response.message = f"Measurement series created: {response.measurement_id}"
        except Exception as exc:
            if self.conn is not None and not self.conn.closed:
                self.conn.rollback()
            response.success = False
            response.message = str(exc)
        return response

    def callback_read_leica(self, request, response):
        """Read one fresh Leica sample without committing anything to PostgreSQL."""
        try:
            timestamp, distance, hz, vt = self.leika.measure_point()
            response.timestamp = int(timestamp)
            response.distance = float(distance)
            response.angle_hz = float(hz)
            response.angle_vt = float(vt)
            response.success = True
            response.message = "Leica sample received (not saved)"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def callback_save_table_point(self, request, response):
        """Read one fresh sample and atomically persist P1/P2/P3 for a selected series.

        Only one row per (measurement_id, point_number). A repeat replaces the previous
        value for the same point; another measurement_id keeps its own independent points.
        """
        try:
            if self.conn is None or self.conn.closed:
                raise RuntimeError("Database is disconnected")
            if request.measurement_id <= 0 or request.point_number not in (1, 2, 3):
                raise ValueError("Valid measurement_id and point_number 1..3 required")
            ensure_measurement_tables(self.conn)
            with self.conn.cursor() as cur:
                cur.execute("SELECT id FROM measurements WHERE id=%s", (request.measurement_id,))
                if cur.fetchone() is None:
                    raise ValueError(f"Measurement series {request.measurement_id} does not exist")
            self.conn.rollback()  # finish validation transaction before sampling
            timestamp, distance, hz, vt = self.leika.measure_point()
            with self.conn.cursor() as cur:
                cur.execute("""
                    INSERT INTO table_calibration_points
                        (measurement_id, point_number, time_of_meas,
                         distance, angleHz, angleVt)
                    VALUES (%s, %s, %s, %s, %s, %s)
                    ON CONFLICT (measurement_id, point_number) DO UPDATE SET
                        time_of_meas=EXCLUDED.time_of_meas,
                        distance=EXCLUDED.distance,
                        angleHz=EXCLUDED.angleHz,
                        angleVt=EXCLUDED.angleVt,
                        created_at=NOW()
                    RETURNING id
                """, (request.measurement_id, request.point_number,
                      timestamp, distance, hz, vt))
                point_id = int(cur.fetchone()[0])
            self.conn.commit()
            response.success = True
            response.message = (f"Table P{request.point_number} committed to "
                                f"measurement series {request.measurement_id}")
            response.point_id = point_id
            response.timestamp = int(timestamp)
            response.distance = float(distance)
            response.angle_hz = float(hz)
            response.angle_vt = float(vt)
            self.get_logger().info(response.message)
        except Exception as exc:
            if self.conn is not None and not self.conn.closed:
                self.conn.rollback()
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f"Table point save failed: {exc}")
        return response

    def read_tracker(self, joints, measurement_id):
        """Acquire one new sample and commit it before returning success."""
        if self.conn is None or self.conn.closed:
            raise RuntimeError("Database is disconnected")
        ensure_measurement_tables(self.conn)
        self.get_logger().info("Waiting for fresh Leica ZMQ measurement")
        timestamp, distance, angle_hz, angle_vt = self.leika.measure_point()
        self.get_logger().info(f"Leica sample received: timestamp={timestamp}, distance={distance}")
        try:
            with self.conn.cursor() as cur:
                cur.execute("""
                    INSERT INTO measurement_data
                    (measurement_id, distance, angleHz, angleVt, time_of_meas,
                     q1, q2, q3, q4, q5, q6, q7)
                    VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
                """, (measurement_id, distance, angle_hz, angle_vt, timestamp, *joints))
            self.conn.commit()
            self.get_logger().info("Leica measurement committed to PostgreSQL")
            return True
        except Exception:
            self.conn.rollback()
            raise

    def callback_save_measurement(self, request, response):
        if not request.save:
            # Controller preflight: prepare schema BEFORE any robot movement.
            try:
                if self.conn is None or self.conn.closed:
                    raise RuntimeError("Database is disconnected")
                ensure_measurement_tables(self.conn)
                response.success = True
                response.message = "PostgreSQL measurement tables ready"
                self.get_logger().info(response.message)
            except Exception as exc:
                response.success = False
                response.message = f"PostgreSQL schema initialization failed: {exc}"
                self.get_logger().error(response.message)
            return response
        if len(request.joints) != 7:
            response.success = False
            response.message = "Exactly seven joint angles are required"
            return response
        if request.measurement_id <= 0:
            response.success = False
            response.message = "A valid measurement_id is required"
            return response
        try:
            self.read_tracker(list(request.joints), request.measurement_id)
            response.success = True
            response.message = "Measurement committed to PostgreSQL"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f"Measurement failed ({type(exc).__name__}): {exc}")
        return response

    def destroy_node(self):
        self.leika.end_connection()
        if self.conn is not None:
            self.conn.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DatabaseNode()
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
