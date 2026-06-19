"""Minimal Wyoming protocol helpers used by the wake-word service."""

import json
import socket


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Socket closed while receiving")
        buf += chunk
    return buf


def wyoming_send_event(
    sock: socket.socket,
    event_type: str,
    data: dict | None = None,
    payload: bytes = b"",
) -> None:
    if data is None:
        data = {}

    header = {
        "type": event_type,
        "data": data,
        "data_length": 0,
        "payload_length": len(payload),
    }
    line = (json.dumps(header, separators=(",", ":")) + "\n").encode("utf-8")
    sock.sendall(line)
    if payload:
        sock.sendall(payload)


def wyoming_recv_event(sock: socket.socket) -> tuple[str, dict, bytes]:
    line = b""
    while not line.endswith(b"\n"):
        chunk = sock.recv(1)
        if not chunk:
            raise ConnectionError("Socket closed while reading event header")
        line += chunk

    header = json.loads(line.decode("utf-8"))
    event_type = header.get("type")
    data = header.get("data") or {}

    data_length = int(header.get("data_length") or 0)
    payload_length = int(header.get("payload_length") or 0)

    if data_length > 0:
        _ = _recv_exact(sock, data_length)

    payload = b""
    if payload_length > 0:
        payload = _recv_exact(sock, payload_length)

    return event_type, data, payload
