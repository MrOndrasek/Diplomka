import pysoem
import time
import zmq
import signal
import sys


class StopPublisher(Exception):
    pass


def _stop_handler(signum, frame):
    raise StopPublisher()


class Connection:
    def __init__(self, port):
        self.port = port
        self._master = pysoem.Master()

        self.ctx = zmq.Context() 
        self.sock = self.ctx.socket(zmq.PUB)
        self.sock.bind("tcp://*:5555")

    def run(self):
        try:
            self._master.open(self.port)
            self._master.config_init()
            if not self._master.slaves:
                raise RuntimeError(f"No EtherCAT slaves found on {self.port}")
            self._master.config_map()

            self._master.send_processdata()
            self._master.receive_processdata(timeout=20000)

            self._master.state = pysoem.OP_STATE
            self._master.write_state()

            self._master.state_check(pysoem.OP_STATE, 50000)
            try:
                while True:
                    print(f"Slave: {self._master.slaves[0].name}")
                    self.timestamp=time.time_ns()
                    self._master.send_processdata()
                    self._master.receive_processdata(timeout=20000)

                    data = self._master.slaves[0].input
                    self.sock.send_multipart([
                        self.timestamp.to_bytes(8,"little"),
                        data
                    ])
                    time.sleep(2)
            except (KeyboardInterrupt, StopPublisher):
                print("Leica publisher stopped", flush=True)
        finally:
            self._master.close()
            self.sock.close()
            self.ctx.term()


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, _stop_handler)
    try:
        Connection("eno4").run()
    except (KeyboardInterrupt, StopPublisher):
        pass
    except Exception as e:
        print(f"Leica error: {e}", file=sys.stderr, flush=True)
        sys.exit(1)
