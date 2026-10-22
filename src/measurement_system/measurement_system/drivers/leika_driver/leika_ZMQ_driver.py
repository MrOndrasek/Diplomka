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
        self.sock.connect("tcp://localhost:5555")


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

            self.get_logger().info(
                f"Discarded {count} old measurements"
            )
    
    def measure_point(self):
        if self.sock is None:
            raise RuntimeError("Leica is not connected")
        
        self.clear_queue()

        if not self.sock.poll(5000, zmq.POLLIN):
             raise TimeoutError("Leica measurement timeout")

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

            return (timestamp,
                    value_distance,
                    value_angle_hz,
                    value_angle_vt)
        
        except (struct.error, ValueError) as e:
            self.logger.error(f"Leica read error:{e}")
            raise
             