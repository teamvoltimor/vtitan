"""src.server.wire – Length-prefixed pickle framing for the TCP model-server protocol.

Single source of truth for pack/unpack so client and server stay in sync.
"""

from __future__ import annotations

import pickle
import socket
import struct

from src.constants import RECV_CHUNK_SIZE


def send(sock: socket.socket, payload: dict) -> None:
    """Serialise *payload* and write it to *sock* with a 4-byte big-endian length prefix."""
    data = pickle.dumps(payload)
    sock.sendall(struct.pack(">I", len(data)) + data)


def recv(sock: socket.socket) -> dict:
    """Read one length-prefixed message from *sock* and return the decoded dict."""
    n = struct.unpack(">I", _recv_bytes(sock, 4))[0]
    return pickle.loads(_recv_bytes(sock, n))  # noqa: S301


def _recv_bytes(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(min(RECV_CHUNK_SIZE, n - len(buf)))
        if not chunk:
            raise ConnectionError("Socket closed before all bytes received")
        buf += chunk
    return buf
