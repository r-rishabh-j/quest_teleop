"""Persistent, length-prefixed JSON over local Unix sockets or wired TCP."""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import selectors
import socket
import struct
import time
from urllib.parse import urlparse

import numpy as np

MAX_MESSAGE = 1024 * 1024
DEFAULT_ENDPOINT = f"unix:///tmp/xarm-teleop-{os.getuid()}.sock"


class RemoteError(RuntimeError):
    """Preserve whether the server already ended control, for safe client cleanup."""

    def __init__(self, response):
        super().__init__(response.get("error", "Server rejected request"))
        self.control_ended = bool(response.get("control_ended"))
        self.reason = response.get("reason")
        self.episode = response.get("episode")


def json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def reject_constant(value):
    raise ValueError(f"Non-finite JSON value: {value}")


def encode(value):
    body = json.dumps(value, default=json_default, allow_nan=False, separators=(",", ":")).encode()
    if len(body) > MAX_MESSAGE:
        raise ValueError("Message exceeds size limit")
    return struct.pack("!I", len(body)) + body


def decode(buffer):
    if len(buffer) < 4:
        return None
    size = struct.unpack("!I", buffer[:4])[0]
    if not 0 < size <= MAX_MESSAGE:
        raise ValueError("Invalid frame size")
    if len(buffer) < size + 4:
        return None
    result = json.loads(bytes(buffer[4:size + 4]), parse_constant=reject_constant)
    del buffer[:size + 4]
    if not isinstance(result, dict):
        raise ValueError("Message must be an object")
    return result


def address(endpoint):
    uri = urlparse(endpoint)
    if uri.scheme == "unix" and uri.path and not uri.netloc:
        return socket.AF_UNIX, uri.path
    if uri.scheme == "tcp" and uri.hostname and uri.port:
        return socket.AF_INET, (uri.hostname, uri.port)
    raise ValueError("Use unix:///absolute/path.sock or tcp://host:port")


class Connection:
    def __init__(self, endpoint=DEFAULT_ENDPOINT, timeout=2.0):
        family, dest = address(endpoint)
        self.socket = socket.socket(family, socket.SOCK_STREAM)
        self.socket.settimeout(timeout)
        if family == socket.AF_INET:
            self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            self.socket.connect(dest)
        except BaseException:
            self.socket.close()
            raise
        self.buffer = bytearray()

    def request(self, value):
        self.socket.sendall(encode(value))
        while True:
            response = decode(self.buffer)
            if response is not None:
                if not response.get("ok"):
                    raise RemoteError(response)
                return response["result"]
            data = self.socket.recv(65536)
            if not data:
                raise ConnectionError("Server disconnected")
            self.buffer.extend(data)

    def close(self):
        self.socket.close()


@dataclass(eq=False)
class Peer:
    socket: socket.socket
    incoming: bytearray = field(default_factory=bytearray)
    outgoing: bytearray = field(default_factory=bytearray)
    touched: float = field(default_factory=time.monotonic)


class SocketServer:
    """Nonblocking I/O: an idle or slow client cannot block the simulation tick."""
    def __init__(self, endpoint, dispatch, disconnected):
        self.selector = selectors.DefaultSelector()
        self.dispatch, self.disconnected = dispatch, disconnected
        self.path = None
        family, dest = address(endpoint)
        self.listener = socket.socket(family, socket.SOCK_STREAM)
        self.listener.setblocking(False)
        if family == socket.AF_INET:
            self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            # Never unlink an existing endpoint: it may belong to a live server.
            self.listener.bind(dest)
            if family == socket.AF_UNIX:
                self.path = Path(dest)
                self.path.chmod(0o600)
            self.listener.listen(8)
            self.selector.register(self.listener, selectors.EVENT_READ)
        except BaseException:
            self.listener.close()
            self.selector.close()
            raise

    def _close_peer(self, peer):
        self.selector.unregister(peer.socket)
        peer.socket.close()
        self.disconnected(peer)

    def poll(self, timeout):
        for key, mask in self.selector.select(max(0, timeout)):
            if key.fileobj is self.listener:
                sock, _ = self.listener.accept()
                sock.setblocking(False)
                if sock.family == socket.AF_INET:
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.selector.register(sock, selectors.EVENT_READ, Peer(sock))
                continue
            peer = key.data
            try:
                if mask & selectors.EVENT_READ:
                    data = peer.socket.recv(65536)
                    if not data:
                        self._close_peer(peer)
                        continue
                    peer.touched = time.monotonic()
                    peer.incoming.extend(data)
                    if len(peer.incoming) > MAX_MESSAGE + 4:
                        raise ValueError("Input buffer exceeded limit")
                    # Clients use request/reply. Reject pipelined floods rather than
                    # allowing old motion to wait behind unbounded queued requests.
                    message = decode(peer.incoming)
                    if message is not None:
                        if peer.incoming or peer.outgoing:
                            raise ValueError("Only one outstanding request is allowed")
                        peer.outgoing.extend(encode(self.dispatch(peer, message)))
                # Flush a small reply immediately. Waiting for another poll
                # would put an extra physics/render tick into every round trip.
                if peer.outgoing:
                    try:
                        sent = peer.socket.send(peer.outgoing)
                        del peer.outgoing[:sent]
                    except BlockingIOError:
                        pass
                events = selectors.EVENT_READ | (selectors.EVENT_WRITE if peer.outgoing else 0)
                self.selector.modify(peer.socket, events, peer)
            except BlockingIOError:
                continue
            except (OSError, ValueError, json.JSONDecodeError):
                self._close_peer(peer)
        for key in list(self.selector.get_map().values()):
            peer = key.data
            if peer is not None and time.monotonic() - peer.touched > 10:
                self._close_peer(peer)

    def close(self):
        for key in list(self.selector.get_map().values()):
            if key.data is not None:
                self._close_peer(key.data)
        self.selector.close()
        self.listener.close()
        if self.path:
            self.path.unlink(missing_ok=True)
