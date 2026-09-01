#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import zmq
import struct
import psycopg2

from measurement_system_interfaces.srv import SaveMeasurement


class DatabaseNode(Node):

    def __init__(self):
        super().__init__("database_node")

        self.recording = False

        # PostgreSQL
        self.conn = self.get_connection()

        if self.conn:
            self.cur = self.conn.cursor()
            self.get_logger().info("Connected to PostgreSQL")
        else:
            self.cur = None
            self.get_logger().error(
                "Could not connect to PostgreSQL"
            )

        # ZMQ
        self.ctx = zmq.Context()

        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.connect("tcp://localhost:5555")
        self.sock.setsockopt(zmq.SUBSCRIBE, b"")

        # Service pro start/stop ukládání
        self.save_service_ = self.create_service(
            SaveMeasurement,
            "save_measurement",
            self.callback_save_measurement
        )

        # Timer pro čtení trackeru
        self.timer = self.create_timer(
            0.001,
            self.read_tracker
        )

        self.get_logger().info(
            "Database node started"
        )


    def read_tracker(self):

        # Nečekej na data blokujícím recv()
        try:
            if not self.sock.poll(0):
                return

            timestamp_data, data = self.sock.recv_multipart(
                zmq.NOBLOCK
            )

        except zmq.Again:
            return

        # Timestamp
        timestamp = int.from_bytes(
            timestamp_data,
            "little"
        )

        # Rozbalení Leica dat
        value_distance = struct.unpack_from(
            "<d", data, 20
        )[0]

        value_angle_hz = struct.unpack_from(
            "<d", data, 12
        )[0]

        value_angle_vt = struct.unpack_from(
            "<d", data, 4
        )[0]

        self.get_logger().debug(
            f"Distance: {value_distance}, "
            f"Hz: {value_angle_hz}, "
            f"Vt: {value_angle_vt}"
        )

        # Ukládání pouze pokud probíhá měření
        if self.recording and self.cur:

            try:
                self.cur.execute(
                    """
                    INSERT INTO measurement_data
                    (
                        distance,
                        angleHz,
                        angleVt,
                        time_of_meas,
                        q1,
                        q2,
                        q3,
                        q4,
                        q5,
                        q6,
                        q7
                    )
                    VALUES (%s, %s, %s, %s,%s,%s,%s,%s,%s,%s,%s)
                    """,
                    (
                        value_distance,
                        value_angle_hz,
                        value_angle_vt,
                        timestamp,
                        self.joints[0],
                        self.joints[1],
                        self.joints[2],
                        self.joints[3],
                        self.joints[4],
                        self.joints[5],
                        self.joints[6]
                        
                    ),
                )

                self.conn.commit()
                self.recording=False
            except Exception as e:

                self.conn.rollback()

                self.get_logger().error(
                    f"Database error: {e}"
                )


    def callback_save_measurement(
        self,
        request,
        response
    ):
        self.joints=request.joints

        self.recording = request.save

        if self.recording:

            self.get_logger().info(
                "Measurement recording STARTED"
            )

            response.success = True
            response.message = "Recording started"

        else:

            self.get_logger().info(
                "Measurement recording STOPPED"
            )

            response.success = True
            response.message = "Recording stopped"

        return response


    def get_connection(self):

        try:

            return psycopg2.connect(
                database="laser_tracker_data",
                user="dohnaon1",
                password="1234",
                host="127.0.0.1",
                port=5432,
            )

        except Exception as e:

            self.get_logger().error(
                f"PostgreSQL connection failed: {e}"
            )

            return None


    def destroy_node(self):

        if self.cur:
            self.cur.close()

        if self.conn:
            self.conn.close()

        self.sock.close()
        self.ctx.term()

        super().destroy_node()


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