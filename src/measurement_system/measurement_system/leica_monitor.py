"""ROS2 Leica stream health monitor. Does not control the tracker."""
import time
import zmq
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class LeicaMonitor(Node):
    def __init__(self):
        super().__init__("leica_monitor")
        self.socket = zmq.Context.instance().socket(zmq.SUB)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.connect("tcp://localhost:5555")
        self.publisher = self.create_publisher(Bool, "/leica/data_available", 10)
        self.last_sample = 0.0
        self.timer = self.create_timer(0.2, self.update)

    def update(self):
        # Avoid blocking the ROS executor or draining an unbounded stream.
        for _ in range(50):
            try:
                frames = self.socket.recv_multipart(flags=zmq.NOBLOCK)
                if len(frames) == 2:
                    self.last_sample = time.monotonic()
            except zmq.Again:
                break
        message = Bool()
        message.data = time.monotonic() - self.last_sample < 2.0
        self.publisher.publish(message)

    def destroy_node(self):
        self.socket.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = LeicaMonitor()
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