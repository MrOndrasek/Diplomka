#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from measurement_system_interfaces.srv import MoveRobot
from measurement_system.drivers.kuka_driver.iiwaPy3 import iiwaPy3

class RobotNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("robot_node") # MODIFY NAME
        self.robot_ = None
        try:
            self.robot_ = iiwaPy3("172.31.1.147")
            self.get_logger().info("Robot connected")

        except Exception as e:
            self.get_logger().error(
                f"Robot connection failed: {e}"
            )
        if self.robot_ is None:
            raise RuntimeError("Robot connection failed; refusing to advertise /move_robot")
        self.move_robot_service_ = self.create_service(MoveRobot,"move_robot",self.callback_move_robot)

     
    def callback_move_robot(self, request: MoveRobot.Request, response: MoveRobot.Response):
            self.get_logger().info(
                f"Received joints: {request.joints}"
            )

            self.get_logger().info(
                f"Velocity: {request.velocity}"
            )
            try:
                velocity=[request.velocity]
                joints= [math.radians(j) for j in request.joints]
                self.get_logger().info("Starting robot movement")
                self.robot_.movePTPJointSpace(joints,velocity)

                response.success = True
                response.message = "Movement finished"
            except Exception as e:
                response.success = False
                response.message = str(e)
            return response
    def closing(self):
        if self.robot_ is not None:
            self.robot_.close()


def main(args=None):
    rclpy.init(args=args)

    node = RobotNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.closing()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
if __name__ == "__main__":
    main()