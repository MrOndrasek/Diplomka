import zmq
import struct
import os
import psycopg2


class data_handling:
    def __init__(self):
        self.ctx = zmq.Context()

    def subscriber(self):
        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.connect("tcp://localhost:5555")
        self.sock.setsockopt(zmq.SUBSCRIBE, b"")

    def publisher(self):
        self.sock_pub = self.ctx.socket(zmq.PUB)
        self.sock_pub.bind("tcp://*:5556")

    def run(self):
        self.conn = self.get_connection()
        self.cur = self.conn.cursor()
        self.subscriber()
        self.publisher()
        while True:

            self.timestamp_data, self.data = self.sock.recv_multipart()
            
            time=int.from_bytes(self.timestamp_data,"little")

            value_distance = struct.unpack_from("<d", self.data, 20)[0]

            value_AngleHz = struct.unpack_from("<d", self.data, 12)[0]

            value_AngleVt = struct.unpack_from("<d", self.data, 4)[0]

            self.cur.execute(
                """
                INSERT INTO measurement_data(distance, angleHz, angleVt, time_of_meas)
                VALUES (%s, %s, %s, %s)
                """,
                (value_distance, value_AngleHz, value_AngleVt, time),
            )
            self.conn.commit()

            packet = struct.pack("<ddd", value_distance, value_AngleHz, value_AngleVt)
            self.sock_pub.send(packet)

            os.system("clear")
            print("distance:", value_distance)
            print("Horizontal_angle:", value_AngleHz)
            print("Vertical_angle:", value_AngleVt)

    def get_connection(self):
        try:
            return psycopg2.connect(
                database="laser_tracker_data",
                user="dohnaon1",
                password="1234",
                host="127.0.0.1",
                port=5432,
            )
        except:
            return False


if __name__ == "__main__":
    data_handling().run()
