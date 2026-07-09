"""Tests for src.server.wire — length-prefixed pickle framing over a socket.

Uses socket.socketpair() for a real, in-process, connected socket pair --
no mocking or actual network I/O needed to exercise send()/recv() end to end.
"""

import socket

import numpy as np
import pytest

from src.server.wire import _recv_bytes, recv, send


@pytest.fixture()
def socket_pair():
    a, b = socket.socketpair()
    yield a, b
    a.close()
    b.close()


class TestSendRecvRoundTrip:
    def test_small_dict_round_trips(self, socket_pair):
        sender, receiver = socket_pair
        payload = {"cmd": "ping"}

        send(sender, payload)
        result = recv(receiver)

        assert result == payload

    def test_payload_with_numpy_array_round_trips(self, socket_pair):
        sender, receiver = socket_pair
        payload = {"image": np.arange(12, dtype=np.uint8).reshape(3, 4)}

        send(sender, payload)
        result = recv(receiver)

        assert np.array_equal(result["image"], payload["image"])

    def test_large_payload_exceeding_one_chunk_round_trips(self, socket_pair):
        sender, receiver = socket_pair
        # recv_chunk_size default is 65536; force a payload well past that
        # so recv() must loop internally to reassemble it.
        payload = {"data": "x" * 200_000}

        send(sender, payload)
        result = recv(receiver, recv_chunk_size=4096)

        assert result == payload

    def test_multiple_messages_in_sequence(self, socket_pair):
        sender, receiver = socket_pair

        send(sender, {"n": 1})
        send(sender, {"n": 2})

        assert recv(receiver) == {"n": 1}
        assert recv(receiver) == {"n": 2}


class TestRecvBytesConnectionHandling:
    def test_socket_closed_before_length_prefix_raises_connection_error(self, socket_pair):
        sender, receiver = socket_pair
        sender.close()

        with pytest.raises(ConnectionError):
            recv(receiver)

    def test_socket_closed_mid_message_raises_connection_error(self, socket_pair):
        sender, receiver = socket_pair
        # Send only a length prefix claiming more data than will ever arrive,
        # then close -- _recv_bytes must not hang or return a short buffer.
        sender.sendall((100).to_bytes(4, "big"))
        sender.close()

        with pytest.raises(ConnectionError):
            recv(receiver)

    def test_recv_bytes_respects_custom_chunk_size(self, socket_pair):
        sender, receiver = socket_pair
        sender.sendall(b"0123456789")

        result = _recv_bytes(receiver, 10, recv_chunk_size=3)

        assert result == b"0123456789"
