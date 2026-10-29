import pysoem
import time
import zmq
# Support direct file execution through pkexec (which strips PYTHONPATH).
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from measurement_system.connection_profiles import load_profiles


class Connection:
    def __init__(self, port, zmq_port=5555):
        self.port = port
        self._master = pysoem.Master()

        self.ctx = zmq.Context() 
        self.sock = self.ctx.socket(zmq.PUB)
        self.sock.bind(f"tcp://*:{int(zmq_port)}")

    def run(self):
        self._master.open(self.port)
        self._master.config_init()
        if len(self._master.slaves) == 0:
            print("No slaves found")
        elif len(self._master.slaves) > 0:
            self._master.config_init()
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
            except KeyboardInterrupt:
                print("ended")
            finally:
                self._master.close()
                self.sock.close()
                self.ctx.term()


if __name__ == "__main__":
    try:
        cfg = load_profiles()["leica"]
        Connection(cfg["interface"], cfg["port"]).run()
    except Exception as e:
        print(f"Leica error: {e}")
