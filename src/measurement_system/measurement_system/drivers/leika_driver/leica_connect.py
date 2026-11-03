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

    def run(self, stop_file=None):
        self._master.open(self.port)
        self._master.config_init()
        if len(self._master.slaves) == 0:
            self._master.close()
            self.sock.close()
            self.ctx.term()
            raise RuntimeError("No EtherCAT slaves found")
        elif len(self._master.slaves) > 0:
            self._master.config_init()
            self._master.config_map()

            self._master.send_processdata()
            self._master.receive_processdata(timeout=20000)

            self._master.state = pysoem.OP_STATE
            self._master.write_state()

            self._master.state_check(pysoem.OP_STATE, 50000)
            try:
                while not (stop_file and Path(stop_file).exists()):
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
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--stop-file", default=None)
    parser.add_argument("--interface", default=None)
    parser.add_argument("--zmq-port", type=int, default=None)
    args = parser.parse_args()
    try:
        # CLI overrides are required for pkexec: root has a DIFFERENT HOME and
        # therefore cannot read the ordinary user's leica.json settings.
        cfg = load_profiles()["leica"] if (args.interface is None or args.zmq_port is None) else {}
        interface = args.interface or cfg["interface"]
        zmq_port = args.zmq_port if args.zmq_port is not None else cfg["port"]
        Connection(interface, zmq_port).run(stop_file=args.stop_file)
    except Exception as e:
        print(f"Leica error: {e}", flush=True)
        raise SystemExit(1)
