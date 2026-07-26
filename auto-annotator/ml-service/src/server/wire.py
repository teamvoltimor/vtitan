"""src.server.wire – Length-prefixed pickle framing for the TCP model-server protocol.

Single source of truth for pack/unpack so client and server stay in sync.
"""

from __future__ import annotations

import pickle
import socket  # noqa: TC003
import struct

from src.config import SERVER_DEFAULT_RECV_CHUNK_SIZE

_DEFAULT_RECV_CHUNK_SIZE: int = SERVER_DEFAULT_RECV_CHUNK_SIZE
"""Fallback chunk size used when no ServerConfig is provided."""


def send(sock: socket.socket, payload: dict) -> None:
    """Serialise *payload* and write it to *sock* with a 4-byte big-endian length prefix."""
    data = pickle.dumps(payload)
    sock.sendall(struct.pack(">I", len(data)) + data)


def recv(sock: socket.socket, recv_chunk_size: int = _DEFAULT_RECV_CHUNK_SIZE) -> dict:
    """Read one length-prefixed message from *sock* and return the decoded dict."""
    n = struct.unpack(">I", _recv_bytes(sock, 4, recv_chunk_size))[0]
    return pickle.loads(_recv_bytes(sock, n, recv_chunk_size))  # noqa: S301


def _recv_bytes(sock: socket.socket, n: int, recv_chunk_size: int = _DEFAULT_RECV_CHUNK_SIZE) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(min(recv_chunk_size, n - len(buf)))
        if not chunk:
            msg = "Socket closed before all bytes received"
            raise ConnectionError(msg)
        buf += chunk
    return buf
