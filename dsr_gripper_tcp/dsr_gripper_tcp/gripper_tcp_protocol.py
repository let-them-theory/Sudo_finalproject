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

# mm <-> raw conversion constants (RH-P12-RN: 0-1150 raw = 0-106 mm)
RAW_MAX = 1150
MM_MAX = 106.0


def mm_to_raw(mm: float) -> int:
    return max(0, min(RAW_MAX, round(mm * RAW_MAX / MM_MAX)))


def raw_to_mm(raw: int) -> float:
    return raw * MM_MAX / RAW_MAX


class Command(IntEnum):
    PING = 1
    INITIALIZE = 2
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

    @property
    def present_position_mm(self) -> float:
        return raw_to_mm(self.present_position)


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


def pack_initialize_payload(goal_current: int) -> bytes:
    return pack_u16(goal_current)


# MOVE payload: pos_mm_x10 (u16) + force_ma (u16) + vel_raw (u32) + accel_raw (u32) + timeout_ms (u32)
# Total: 16 bytes
def pack_move_payload(position_mm: float, force_ma: int, vel_raw: int, accel_raw: int, timeout_ms: int) -> bytes:
    pos_mm_x10 = max(0, min(1060, round(position_mm * 10)))
    return struct.pack(">HHIII", pos_mm_x10, int(force_ma), int(vel_raw), int(accel_raw), int(timeout_ms))


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
