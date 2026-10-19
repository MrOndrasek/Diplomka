import zmq
import struct


class Leika_ZMQ_driver():

    def __init__(self):
        pass

    def connection(self):
        self.ctx=zmq.Context()
        self.sock=self.ctx.socket(zmq.SUB)
        self.sock.connect("tcp://localhost:5555")
        self.sock.setsockopt(zmq.SUBSCRIBE,b"")
        return None

    def end_connection(self):
        self.sock.close()
        self.ctx.term()

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
    
    def measure_point(self):
        self.clear_queue()

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

            return timestamp,value_distance,value_angle_hz,value_angle_vt
        
        except Exception as e:

             self.conn.rollback()
             self.get_logger().error(
                  "Leika read error"
             )
             return False