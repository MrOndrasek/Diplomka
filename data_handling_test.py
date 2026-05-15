import zmq
import struct
import os

ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.connect("tcp://localhost:5555")
sock.setsockopt(zmq.SUBSCRIBE, b"")

while True:
    data = sock.recv()

    value_distance = struct.unpack_from("<d", data, 20)[0]

    value_AngleHz = struct.unpack_from("<d", data, 12)[0]

    value_AngleVt = struct.unpack_from("<d", data, 4)[0]

    os.system("clear")

    print("distance:", value_distance)
    print("Horizontal_angle:", value_AngleHz)
    print("Vertical_angle:", value_AngleVt)
