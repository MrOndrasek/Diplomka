import zmq
import struct
import os


class data_handling:
    def __init__(self):
        self.ctx = zmq.Context()

    def subscriber(self):
        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.connect("tcp://localhost:5555")
        self.sock.setsockopt(zmq.SUBSCRIBE, b"")


    def run(self):
        self.subscriber()
        while True:

            self.data = self.sock.recv()
            
            self.data_decode = bytes.fromhex(self.data.hex())

            for i in range(0, len(self.data_decode), 4):
                print(i, struct.unpack_from('<f', self.data_decode, i)[0])
            
            """for i in range(0, len(self.data)-8, 1):
                try:
                    val = struct.unpack_from('<d', self.data, i)[0]
                    if val != 0:
                        print(i, val)
                except:
                    pass
            """

if __name__ == "__main__":
    data_handling().run()










