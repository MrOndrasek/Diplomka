#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from measurement_system_interfaces.srv import MoveRobot, GetRobotJoints
from measurement_system.drivers.kuka_driver.iiwaPy3 import iiwaPy3
from measurement_system.connection_profiles import selected_robot

class RobotNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("robot_node") # MODIFY NAME
        self.robot_ = None
        try:
            robot_settings = selected_robot()
            if robot_settings is None:
                raise RuntimeError("No robot selected in Settings")
            self.get_logger().info(f"Connecting to {robot_settings['name']} at {robot_settings['host']}:{robot_settings['port']}")
            self.robot_ = iiwaPy3(robot_settings["host"], port=int(robot_settings["port"]))
            self.get_logger().info("Robot connected")

        except Exception as e:
            self.get_logger().error(
                f"Robot connection failed: {e}"
            )
        if self.robot_ is None:
            raise RuntimeError("Robot connection failed; refusing to advertise /move_robot")
        self.read_joints_service_ = self.create_service(GetRobotJoints, "/get_robot_joints", self.callback_get_joints)
        self.move_robot_service_ = self.create_service(MoveRobot,"move_robot",self.callback_move_robot)

     
    def callback_move_robot(self, request: MoveRobot.Request, response: MoveRobot.Response):
            self.get_logger().info(
                f"Received joints: {request.joints}"
            )

            self.get_logger().info(
                f"Velocity: {request.velocity}"
            )
            response.actual_joints = []
            try:
                velocity=[request.velocity]
                joints= [math.radians(j) for j in request.joints]
                self.get_logger().info("Starting robot movement")
                self.robot_.movePTPJointSpace(joints,velocity)

                # KST/iiwaPy3 returns joint positions in radians.
                # CSV and SaveMeasurement use degrees: convert only on output.
                measured_rad = self.robot_.getJointsPos()
                if len(measured_rad) != 7:
                    raise RuntimeError(
                        f"KUKA returned {len(measured_rad)} joint positions instead of 7"
                    )
                measured_deg = [math.degrees(float(q)) for q in measured_rad]
                if not all(math.isfinite(q) for q in measured_deg):
                    raise RuntimeError("KUKA returned non-finite joint positions")

                response.actual_joints = measured_deg
                response.success = True
                response.message = "Movement finished; actual joints read from robot"
                self.get_logger().info(f"Actual joints [deg]: {measured_deg}")
            except Exception as e:
                response.success = False
                response.message = str(e)
                self.get_logger().error(f"KUKA movement failed: {e}")
            return response
    def callback_get_joints(self, request, response):
        """Read seven measured joint angles from KUKA, returned in degrees."""
        try:
            values = list(self.robot_.getJointsPos())
            if len(values) != 7 or not all(math.isfinite(v) for v in values):
                raise ValueError(f"Invalid KUKA joint readout: {values}")
            response.joints = [math.degrees(v) for v in values]
            response.success = True
            response.message = "Current KUKA joints read"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
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