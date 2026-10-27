#!/usr/bin/env python3
"""ROS2 endpoint for one Leica sample and an atomic PostgreSQL insert."""
import rclpy
from rclpy.node import Node
from measurement_system_interfaces.srv import SaveMeasurement
from measurement_system.drivers.db_setup.databaze import get_connection as existing_db_connection
from measurement_system.drivers.leika_driver.leika_ZMQ_driver import Leika_ZMQ_driver


class DatabaseNode(Node):
    def __init__(self):
        super().__init__("database_node")
        self.conn = self.get_connection()
        self.leika = Leika_ZMQ_driver()
        self.leika.connection()
        self.get_logger().info("Leica subscriber initialized on tcp://127.0.0.1:5555")
        self.save_service_ = self.create_service(
            SaveMeasurement, "/save_measurement", self.callback_save_measurement
        )
        if self.conn is None:
            self.get_logger().warning("PostgreSQL unavailable; measurement requests will fail")

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

    def read_tracker(self, joints):
        """Acquire one new sample and commit it before returning success."""
        if self.conn is None or self.conn.closed:
            raise RuntimeError("Database is disconnected")
        self.get_logger().info("Waiting for fresh Leica ZMQ measurement")
        timestamp, distance, angle_hz, angle_vt = self.leika.measure_point()
        self.get_logger().info(f"Leica sample received: timestamp={timestamp}, distance={distance}")
        try:
            with self.conn.cursor() as cur:
                cur.execute("""
                    INSERT INTO measurement_data
                    (distance, angleHz, angleVt, time_of_meas,
                     q1, q2, q3, q4, q5, q6, q7)
                    VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
                """, (distance, angle_hz, angle_vt, timestamp, *joints))
            self.conn.commit()
            self.get_logger().info("Leica measurement committed to PostgreSQL")
            return True
        except Exception:
            self.conn.rollback()
            raise

    def callback_save_measurement(self, request, response):
        if not request.save:
            response.success = True
            response.message = "Measurement skipped"
            return response
        if len(request.joints) != 7:
            response.success = False
            response.message = "Exactly seven joint angles are required"
            return response
        try:
            self.read_tracker(list(request.joints))
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
