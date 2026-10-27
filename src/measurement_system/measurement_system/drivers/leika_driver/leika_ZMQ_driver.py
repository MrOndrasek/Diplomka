import zmq
import struct
import logging
import time


class Leika_ZMQ_driver():

    def __init__(self):
        self.ctx = None
        self.sock = None

        self.logger = logging.getLogger(__name__)

    def connection(self):
        if self.sock is not None:
             return
        
        self.ctx=zmq.Context()
        self.sock=self.ctx.socket(zmq.SUB)

        self.sock.setsockopt(zmq.SUBSCRIBE,b"")
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.setsockopt(zmq.RCVTIMEO, 5000)
        self.sock.connect("tcp://127.0.0.1:5555")


    def end_connection(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

        if self.ctx is not None: 
            self.ctx.term()
            self.ctx = None

    def clear_queue(self):
            if self.sock is None:
                raise RuntimeError("Leica is not connected")

            count=0
            start = time.monotonic()

            while time.monotonic() - start < 0.1:
                try:
                    self.sock.recv_multipart(flags=zmq.NOBLOCK)
                    count+=1
    
                except zmq.Again:
                    break

            self.logger.info(
                f"Discarded {count} old measurements"
            )
    
    def measure_point(self):

            if self.sock is None:
                raise RuntimeError("Leica is not connected")

            self.logger.warning("Leica: clearing queue")

            self.clear_queue()

            self.logger.warning("Leica: waiting for new measurement")

            start = time.monotonic()

            if not self.sock.poll(5000, zmq.POLLIN):
                elapsed = time.monotonic() - start

                self.logger.error(
                    f"Leica timeout after {elapsed:.2f} seconds"
                )

                raise TimeoutError("Leica measurement timeout")

            self.logger.warning("Leica: new message received")

            parts = self.sock.recv_multipart()

            self.logger.warning(
                f"Leica packet: {len(parts)} parts, "
                f"sizes: {[len(p) for p in parts]}"
            )

            if len(parts) != 2:
                raise ValueError(
                    f"Expected 2 Leica packet parts, received {len(parts)}"
                )

            timestamp_data, data = parts

            if len(timestamp_data) != 8 or len(data) < 28:
                raise ValueError("Invalid Leica packet size")

            timestamp = int.from_bytes(
                timestamp_data,
                "little"
            )

            value_distance = struct.unpack_from(
                "<d", data, 20
            )[0]

            value_angle_hz = struct.unpack_from(
                "<d", data, 12
            )[0]

            value_angle_vt = struct.unpack_from(
                "<d", data, 4
            )[0]

            self.logger.warning(
                f"Leica measurement received: "
                f"distance={value_distance}, "
                f"angleHz={value_angle_hz}, "
                f"angleVt={value_angle_vt}"
            )

            return (
                timestamp,
                value_distance,
                value_angle_hz,
                value_angle_vt
            )
        
        except (struct.error, ValueError) as e:
            self.logger.error(f"Leica read error:{e}")
            raise

