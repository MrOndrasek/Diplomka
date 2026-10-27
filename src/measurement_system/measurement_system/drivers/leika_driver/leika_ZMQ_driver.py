"""Leica subscriber. Uses one persistent socket, only after ZMQ publisher is running."""
import logging
import struct
import time
import zmq


class Leika_ZMQ_driver:
    def __init__(self, endpoint='tcp://127.0.0.1:5555', timeout_ms=10000):
        self.ctx = None
        self.sock = None
        self.endpoint = endpoint
        self.timeout_ms = timeout_ms
        self.logger = logging.getLogger(__name__)

    def connection(self):
        if self.sock is not None:
            return
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.setsockopt(zmq.SUBSCRIBE, b'')
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.connect(self.endpoint)
        self.logger.info('Leica ZMQ SUB connected to %s (waiting for publisher)', self.endpoint)

    def end_connection(self):
        if self.sock is not None:
            self.sock.close(linger=0)
            self.sock = None
        if self.ctx is not None:
            self.ctx.term()
            self.ctx = None

    def clear_queue(self):
        if self.sock is None:
            raise RuntimeError('Leica ZMQ socket not initialized')
        count = 0
        while self.sock.poll(0, zmq.POLLIN):
            self.sock.recv_multipart()
            count += 1
        self.logger.debug('Discarded %d queued Leica samples', count)
        return count

    def measure_point(self):
        if self.sock is None:
            raise RuntimeError('Leica ZMQ socket not initialized')
        # Discard only already available samples; request a NEW packet.
        dropped = self.clear_queue()
        started = time.monotonic()
        if not self.sock.poll(self.timeout_ms, zmq.POLLIN):
            raise TimeoutError(
                f'Leica ZMQ timeout ({self.timeout_ms} ms) on {self.endpoint}; '
                f'discarded {dropped} queued samples. Check Leica publisher.'
            )
        parts = self.sock.recv_multipart()
        if len(parts) != 2:
            raise ValueError(f'Leica ZMQ expected 2 frames; got {len(parts)}')
        timestamp_data, data = parts
        if len(timestamp_data) != 8 or len(data) < 28:
            raise ValueError('Invalid Leica packet: frame sizes '
                             f'{len(timestamp_data)}, {len(data)}')
        timestamp = int.from_bytes(timestamp_data, 'little')
        distance = struct.unpack_from('<d', data, 20)[0]
        angle_hz = struct.unpack_from('<d', data, 12)[0]
        angle_vt = struct.unpack_from('<d', data, 4)[0]
        self.logger.info('Leica sample received in %.3fs', time.monotonic()-started)
        return timestamp, distance, angle_hz, angle_vt
