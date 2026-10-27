"""Capture three Leica samples defining a table frame (no transforms calculated)."""
import logging
import os
import psycopg2
from measurement_system.drivers.leika_driver.leika_ZMQ_driver import Leika_ZMQ_driver


class TablePlane:
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.points = {}
        self.leika = Leika_ZMQ_driver()
        self.conn = None
        self.cur = None
        self.leika.connection()
        try:
            self.conn = psycopg2.connect(
                dbname=os.getenv("PGDATABASE", "laser_tracker_data"),
                user=os.getenv("PGUSER", "dohnaon1"),
                host=os.getenv("PGHOST", "127.0.0.1"),
                port=os.getenv("PGPORT", "5432"), connect_timeout=5)
            self.cur = self.conn.cursor()
        except Exception:
            self.leika.end_connection()
            raise

    def measure_table_point(self, point_id):
        if point_id not in (1, 2, 3):
            raise ValueError("Expected P1, P2 or P3 as integers 1, 2, 3")
        self.points[point_id] = self.leika.measure_point()
        return self.points[point_id]

    def send_to_database(self):
        if set(self.points) != {1, 2, 3}:
            raise ValueError("All three table points must be measured")
        try:
            for point_id, (timestamp, distance, hz, vt) in sorted(self.points.items()):
                self.cur.execute("""
                    INSERT INTO table_plane_points (point, distance, angleHz, angleVt)
                    VALUES (%s, %s, %s, %s)
                """, (point_id, distance, hz, vt))
            self.conn.commit()
            return True
        except Exception as exc:
            self.conn.rollback()
            self.logger.error("Table points insert failed: %s", exc)
            return False

    def close(self):
        self.leika.end_connection()
        if self.cur is not None:
            self.cur.close()
        if self.conn is not None:
            self.conn.close()


# Compatibility with older imports.
table_plane = TablePlane
