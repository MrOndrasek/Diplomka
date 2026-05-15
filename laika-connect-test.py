import pysoem
import struct
import time
import os
import binascii
import zmq

IFNAME = "eno2"
master = pysoem.Master()
master.open(IFNAME)
master.config_init()

distance_list = []
ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.bind("tcp://*:5555")

if len(master.slaves) == 0:
    print("No slaves found")

elif len(master.slaves) > 0:
    master.config_init()
    master.config_map()

    master.send_processdata()
    master.receive_processdata(timeout=20000)

    master.state = pysoem.OP_STATE
    master.write_state()

    master.state_check(pysoem.OP_STATE, 50000)


while True:
    try:
        slave = master.slaves[0]
        print(f"Slave: {slave.name}")
        print(master.read_state())

        master.send_processdata()
        master.receive_processdata(timeout=20000)

        data = slave.input
        sock.send(data)
        # os.system("clear")
        time.sleep(0.1)
        # print(len(distance_list))
        # break
    except KeyboardInterrupt:
        print("skipped")
        print(master.config_init())
        master.state = pysoem.INIT_STATE
        master.write_state()
        print(distance_list)
        time.sleep(1)
