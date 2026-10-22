#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import zmq
import psycopg2

from measurement_system_interfaces.srv import SaveMeasurement
from measurement_system.measurement_system.drivers.leika_driver.leika_ZMQ_driver import Leika_ZMQ_driver


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
        self.leika=Leika_ZMQ_driver()
        self.leika.connection()

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
        self.leika.clear_queue()

        try:
            timestamp,distance,angle_hz,angle_vt=(
                self.leika.measure_point()
            )   
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
                    distance,
                    angle_hz,
                    angle_vt,
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


    def destroy_node(self):

        if self.cur:
            self.cur.close()

        if self.conn:
            self.conn.close()

        Leika_ZMQ_driver.end_connection()
        
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