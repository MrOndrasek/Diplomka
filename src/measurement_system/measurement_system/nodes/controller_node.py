#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from measurement_system_interfaces.srv import MoveRobot
from measurement_system_interfaces.srv import SaveMeasurement
import os



class ControllerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("controller_node") # MODIFY NAME
        self.client_ = self.create_client(MoveRobot, "move_robot")
        self.timer = self.create_timer(
            1.0,
            self.call_move_robot
        )
        self.client_meas=self.create_client(SaveMeasurement,"save_measurement")

        self.request_sent = False
        self.request_meas_sent=False

        self.positions = [
                        [-5, 5, 5, 5, 5, 5, 5],
                        [-5, 5, 5, 5, 5, 5, 5],
                        [-5, 5, 5, 5, 5, 5, 5],
                        [0, 0, 0, 0, 0, 0, 0],
             
                    ]
        self.position_index=0
        
    def call_move_robot(self):
            if self.position_index == len(self.positions):
                                self.get_logger().info("All positions completed")
                
                                self.timer.cancel()
                
                                raise SystemExit
            if self.request_sent:
                return
            if not self.client_.wait_for_service(1.0):
                self.get_logger().warn("Waiting for robot_node...")
                return
            
            request = MoveRobot.Request()
            request.joints = self.positions[self.position_index]
            request.velocity=0.3
            self.get_logger().info(
            f"Sending robot movement request: {self.position_index}"
            )
            future = self.client_.call_async(request)
            future.add_done_callback(self.callback_move_robot_response)
            self.request_sent = True

    def callback_move_robot_response(self,future):
            try:
                response = future.result()

                self.get_logger().info(
                    f"Success: {response.success}"
                )

                self.get_logger().info(
                    f"Message: {response.message}"
                )
                if response.success:
                    self.call_save_measurement()
                    

            except Exception as e:
                self.get_logger().error(
                    f"Service failed: {e}"
                )
            finally:
                 self.request_sent=False

    def call_save_measurement(self):
            if self.request_meas_sent:
                    return
            if not self.client_meas.wait_for_service(1.0):
                    self.get_logger().warn("Waiting for database node...")
                    return
            request = SaveMeasurement.Request()
            request.save=True
            request.joints=self.positions[self.position_index]
            self.get_logger().info(
                "Sending saving request"
            )
            future = self.client_meas.call_async(request)
            future.add_done_callback(self.callback_save_measurement_response) 
            self.request_meas_sent=True

    def callback_save_measurement_response(self,future):
            try:
                response = future.result()
          
                self.get_logger().info(
                    f"Success: {response.success}"
                          )
          
                self.get_logger().info(
                    f"Message: {response.message}"
                          )
                if response.success:

            # Poslední pozice byla právě uložena
                    if self.position_index == len(self.positions) - 1:

                        self.get_logger().info(
                            "Last measurement saved"
                        )

                        self.get_logger().info(
                            "All positions completed"
                        )

                        self.timer.cancel()

                        raise SystemExit

                    # Teprve nyní přejdeme na další pozici
                    self.position_index += 1
                    

          
            except Exception as e:
                self.get_logger().error(
                    f"Service failed: {e}"
                    )
            finally:
                self.request_meas_sent=False

def main(args=None):
    rclpy.init(args=args)

    node = ControllerNode()

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