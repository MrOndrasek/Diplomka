#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from measurement_system_interfaces.srv import MoveRobot
from measurement_system_interfaces.srv import SaveMeasurement
import os

from measurement_system.measurement.position_importer import load_positions


import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node

from measurement_system_interfaces.srv import (
    MoveRobot,
    SaveMeasurement,
    StartMeasurement
)

from measurement_system.measurement_status import (
    MeasurementStatusPublisher
)


class State(Enum):
    IDLE = auto()
    MOVING = auto()
    WAIT_MOVE = auto()
    SETTLING = auto()
    MEASURING = auto()
    WAIT_MEASUREMENT = auto()
    FINISHED = auto()
    ERROR = auto()


class MeasurementController(Node):

    def __init__(self):
        super().__init__("measurement_controller")

        self.state = State.IDLE
        self.status_publisher=MeasurementStatusPublisher(self)

        self.positions = []
        self.position_index = 0

        self.future = None
        self.settle_until = 0.0

        self.start_service = self.create_service(
        StartMeasurement,
        "start_measurement",
        self.callback_start_measurement
        )

        self.robot_client = self.create_client(
            MoveRobot, "move_robot"
        )

        self.measurement_client = self.create_client(
            SaveMeasurement, "save_measurement"
        )

        # Stavový automat běží 10x za sekundu
        self.timer = self.create_timer(
            0.1, self.update
        )

    def start(self, positions):
        if self.state not in (
            State.IDLE, State.FINISHED, State.ERROR
        ):
            return False

        if not positions:
            return False

        self.positions = positions
        self.position_index = 0
        self.future = None

        self.state = State.MOVING
        return True

    
    def callback_start_measurement(self, request, response):

        try:
            positions, point_ids = load_positions(
                request.csv_path
            )

            success = self.start_measurement(
                positions,
                point_ids
            )

            response.success = success
            response.message = (
                "Measurement started"
                if success
                else "Measurement already running"
            )

        except Exception as e:
            response.success = False
            response.message = str(e)

        return response


    def start_measurement(self, positions, point_ids):

        if self.state not in (
            State.IDLE,
            State.FINISHED,
            State.ERROR
        ):
            return False

        if not positions:
            return False

        self.positions = positions
        self.point_ids = point_ids

        self.position_index = 0
        self.future = None

        self.state = State.MOVING

        self.get_logger().info(
            f"Starting measurement: {len(positions)} points"
        )

        return True

    def update(self):

        if self.state == State.MOVING:

            if not self.robot_client.service_is_ready():
                self.fail("Robot service unavailable")
                return

            joints = self.positions[self.position_index]

            request = MoveRobot.Request()
            request.joints = [float(q) for q in joints]
            request.velocity = 0.1

            self.future = self.robot_client.call_async(
                request
            )

            self.state = State.WAIT_MOVE

        elif self.state == State.WAIT_MOVE:

            if not self.future.done():
                return

            try:
                response = self.future.result()
                if not response.success:
                    self.fail(response.message)
                    return
            except Exception as e:
                self.fail(str(e))
                return

            # Příklad ustalovací prodlevy
            self.settle_until = time.monotonic() + 1.0
            self.state = State.SETTLING

        elif self.state == State.SETTLING:

            if time.monotonic() >= self.settle_until:
                self.state = State.MEASURING

        elif self.state == State.MEASURING:

            if not self.measurement_client.service_is_ready():
                self.fail("Measurement service unavailable")
                return

            request = SaveMeasurement.Request()

            request.save = True
            request.joints = [
                float(q)
                for q in self.positions[self.position_index]
            ]

            self.future = (
                self.measurement_client.call_async(request)
            )

            self.state = State.WAIT_MEASUREMENT

        elif self.state == State.WAIT_MEASUREMENT:

            if not self.future.done():
                return

            try:
                response = self.future.result()
                if not response.success:
                    self.fail(response.message)
                    return
            except Exception as e:
                self.fail(str(e))
                return

            self.position_index += 1

            if self.position_index >= len(self.positions):
                self.state = State.FINISHED
                self.get_logger().info(
                    "Measurement completed"
                )
            else:
                self.state = State.MOVING

        self.status_publisher.publish(
            state=self.state.name,
            completed=self.position_index,
            total=len(self.positions)
        )

    def fail(self, message):
            self.get_logger().error(message)
            self.state = State.ERROR

            self.status_publisher.publish(
                state="ERROR",
                completed=self.position_index,
                total=len(self.positions),
                message=message
            )




def main(args=None):
    rclpy.init(args=args)

    node = MeasurementController()

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