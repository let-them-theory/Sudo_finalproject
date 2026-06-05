from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import socket
import struct


MAGIC = b"GP"
VERSION = 1
HEADER_STRUCT = struct.Struct(">2sBBHH")
HEADER_SIZE = HEADER_STRUCT.size
STATE_STRUCT = struct.Struct(">BBBBhhii")


class Command(IntEnum):
    PING = 1
    INITIALIZE = 2
    SET_CONFIG = 3
    MOVE = 4
    READ_STATE = 5
    SHUTDOWN = 6
    SET_TORQUE = 7


class StatusCode(IntEnum):
    OK = 0
    BAD_PACKET = 1
    BAD_COMMAND = 2
    IO_ERROR = 3
    TIMEOUT = 4
    RANGE_ERROR = 5
    NOT_READY = 6


@dataclass(slots=True)
class GripperState:
    status: int
    moving: int
    moving_status: int
    present_current: int
    present_temperature: int
    present_velocity: int
    present_position: int
    torque_enabled: bool = False

    @property
    def in_position(self) -> bool:
        return bool(self.moving_status & 0x01)


def build_packet(command: int, sequence: int, payload: bytes = b"") -> bytes:
    return HEADER_STRUCT.pack(MAGIC, VERSION, int(command), int(sequence), len(payload)) + payload


def recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = size
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("TCP connection closed while receiving data.")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def recv_packet(sock: socket.socket) -> tuple[int, int, bytes]:
    header = recv_exact(sock, HEADER_SIZE)
    magic, version, command, sequence, payload_size = HEADER_STRUCT.unpack(header)
    if magic != MAGIC:
        raise ValueError(f"Unexpected packet magic: {magic!r}")
    if version != VERSION:
        raise ValueError(f"Unsupported protocol version: {version}")
    payload = recv_exact(sock, payload_size) if payload_size else b""
    return command, sequence, payload


def pack_u16(value: int) -> bytes:
    return struct.pack(">H", int(value))


def pack_u32(value: int) -> bytes:
    return struct.pack(">I", int(value))


def pack_initialize_payload(goal_current: int) -> bytes:
    return pack_u16(goal_current)


def pack_config_payload(goal_current: int, profile_velocity: int, profile_acceleration: int) -> bytes:
    return pack_u16(goal_current) + pack_u32(profile_velocity) + pack_u32(profile_acceleration)


def pack_move_payload(goal_position: int, timeout_ms: int) -> bytes:
    return pack_u32(goal_position) + pack_u32(timeout_ms)


def pack_torque_payload(enabled: bool) -> bytes:
    return pack_u16(1 if enabled else 0)


def unpack_state_payload(payload: bytes) -> GripperState:
    if len(payload) != STATE_STRUCT.size:
        raise ValueError(f"Expected {STATE_STRUCT.size} bytes, received {len(payload)} bytes.")
    status, moving, moving_status, torque_flag, current, temperature, velocity, position = STATE_STRUCT.unpack(payload)
    return GripperState(
        status=status,
        moving=moving,
        moving_status=moving_status,
        present_current=current,
        present_temperature=temperature,
        present_velocity=velocity,
        present_position=position,
        torque_enabled=bool(torque_flag & 0x01),
    )
