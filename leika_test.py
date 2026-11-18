import zmq

ctx=zmq.Context()
sock=ctx.socket(zmq.SUB)
sock.setsockopt(zmq.SUBSCRIBE,b"")
sock.connect("tcp://127.0.0.1:5555")

print("Cekam na leika...")

for i in range(3):
    if sock.poll(10000,zmq.POLLIN):
        data=sock.recv_multipart()
        print(f"Mereni {i+1}:{data}")
    else:
        print("TIMEOUT-Zadna data")