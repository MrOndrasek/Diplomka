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

        self.get_logger().info(
            "Database node started"
        )


    def read_tracker(self):

        #vycisti stara data
        self.clear_queue()

        #timeout pro tracker
        if not self.sock.poll(5000):
            self.get_logger().error("Leika timeout")
            return False

        try:
            timestamp_data, data=self.sock.recv_multipart()
            
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

            # Ukládání pouze pokud probíhá měření

            
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
                    *self.joints
                    
                ),
            )

            self.conn.commit()
            return False
        
        except Exception as e:

                    self.conn.rollback()

                    self.get_logger().error(
                        f"Database error: {e}"
                    )
                    return False
            


    def callback_save_measurement(
        self,
        request,
        response
    ):
        if not request.save:
            response.success=True
            response.message="Measurrement skipped"
            return response

        if self.cur is None:
            response.success=False
            response.message="Database unavaiable"
            return response

        if len(request.joints) != 7:
            response.success=False
            response.message="Invalid joint position"
            return response

        self.joints=list(request.joints)

        success=self.read_tracker()


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

    def clear_queue(self):
        count=0
        while True:
            try:
                self.sock.recv_multipart(flags=zmq.NOBLOCK)
                count+=1

            except zmq.Again:
                break
        self.get_logger().info(
            f"Discarded {count} old measurements"
        )


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