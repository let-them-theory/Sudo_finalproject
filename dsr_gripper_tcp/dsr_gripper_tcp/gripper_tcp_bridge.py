from __future__ import annotations

import socket
import textwrap
import time

from dsr_gripper_tcp.gripper_tcp_protocol import (
    Command,
    GripperState,
    StatusCode,
    build_packet,
    pack_initialize_payload,
    pack_move_payload,
    pack_torque_payload,
    recv_packet,
    unpack_state_payload,
)


class GripperBridge:
    """TCP socket connection to the Doosan DRL gripper server."""

    def __init__(self, host: str, port: int = 20002) -> None:
        self._host = host
        self._port = port
        self._sock: socket.socket | None = None
        self._seq = 1

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def connect(self, timeout: float = 30.0) -> None:
        deadline = time.monotonic() + timeout
        last_log = 0.0
        while time.monotonic() < deadline:
            try:
                s = socket.create_connection((self._host, self._port), timeout=1.0)
                s.settimeout(10.0)
                self._sock = s
                return
            except OSError:
                now = time.monotonic()
                if now - last_log >= 3.0:
                    last_log = now
                time.sleep(0.1)
        raise RuntimeError(
            f"Could not connect to gripper TCP server "
            f"{self._host}:{self._port} within {timeout:.0f}s"
        )

    def close(self) -> None:
        if self._sock:
            try:
                self._send(Command.SHUTDOWN, b"", timeout=2.0)
            except Exception:
                pass
            self._sock.close()
            self._sock = None
            self._seq = 1

    def reset(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
            self._seq = 1

    def initialize(self, goal_current: int = 400, timeout: float = 40.0) -> GripperState:
        return self._send(Command.INITIALIZE, pack_initialize_payload(goal_current), timeout=timeout)

    def move_to(
        self,
        position_mm: float,
        goal_current: int,
        profile_velocity: int,
        profile_acceleration: int,
        timeout_sec: float = 10.0,
    ) -> GripperState:
        """Move gripper to position_mm (0.0-106.0mm) with given force/velocity settings."""
        timeout_ms = max(0, int(timeout_sec * 1000))
        payload = pack_move_payload(position_mm, goal_current, profile_velocity, profile_acceleration, timeout_ms)
        return self._send(Command.MOVE, payload, timeout=max(timeout_sec + 2.0, 5.0))

    def set_torque(self, enabled: bool) -> GripperState:
        return self._send(Command.SET_TORQUE, pack_torque_payload(enabled))

    def read_state(self, timeout: float = 2.0) -> GripperState:
        return self._send(Command.READ_STATE, b"", timeout=timeout)

    def _send(self, command: Command, payload: bytes, timeout: float = 10.0) -> GripperState:
        if self._sock is None:
            raise RuntimeError("Gripper bridge not connected")
        packet = build_packet(int(command), self._seq, payload)
        self._sock.settimeout(timeout)
        self._sock.sendall(packet)
        try:
            _, _, resp_payload = recv_packet(self._sock)
        except ValueError as exc:
            raise RuntimeError(f"Bad packet: {exc}") from exc
        self._seq = self._seq % 65535 + 1
        state = unpack_state_payload(resp_payload)
        if state.status != StatusCode.OK:
            raise RuntimeError(f"{command.name} failed with status {int(state.status)}")
        return state


def build_drl_script(
    port: int = 20002,
    slave_id: int = 1,
    baudrate: int = 57600,
    goal_current: int = 400,
    profile_velocity: int = 1500,
    profile_acceleration: int = 1000,
    position_tolerance: int = 5,
) -> str:
    return textwrap.dedent(f"""
        CMD_PING = 1
        CMD_INITIALIZE = 2
        CMD_MOVE = 4
        CMD_READ_STATE = 5
        CMD_SHUTDOWN = 6
        CMD_SET_TORQUE = 7

        STATUS_OK = 0
        STATUS_BAD_PACKET = 1
        STATUS_BAD_COMMAND = 2
        STATUS_IO_ERROR = 3
        STATUS_TIMEOUT = 4
        STATUS_RANGE_ERROR = 5
        STATUS_NOT_READY = 6

        HEADER_SIZE = 8
        POLL_WAIT_SEC = 0.05
        POSITION_TOLERANCE = {position_tolerance}

        # RH-P12-RN Modbus RTU register map
        ADDR_TORQUE_ENABLE = 256
        ADDR_GOAL_CURRENT = 275
        ADDR_PROFILE_ACCELERATION = 278
        ADDR_PROFILE_VELOCITY = 280
        ADDR_GOAL_POSITION = 282
        ADDR_MOVING_STATUS = 285
        ADDR_PRESENT_CURRENT = 287
        ADDR_PRESENT_VELOCITY = 288
        ADDR_PRESENT_POSITION = 290
        ADDR_PRESENT_TEMPERATURE = 297

        # mm <-> raw conversion: 0-106mm = 0-1150 raw
        RAW_MAX = 1150
        MM_MAX_X10 = 1060

        g_slaveid = {slave_id}
        g_goal_current = {goal_current}
        g_profile_velocity = {profile_velocity}
        g_profile_acceleration = {profile_acceleration}
        g_last_accel = -1
        g_last_velocity = -1
        g_sock = None
        g_ready = False

        def mm_x10_to_raw(pos_mm_x10):
            return int(pos_mm_x10 * RAW_MAX / MM_MAX_X10)

        def raw_to_mm_x10(raw):
            return int(raw * MM_MAX_X10 / RAW_MAX)

        def modbus_set_slaveid(slaveid):
            global g_slaveid
            g_slaveid = slaveid

        def modbus_fc03(startaddress, cnt):
            global g_slaveid
            data = (g_slaveid).to_bytes(1, byteorder='big')
            data += (3).to_bytes(1, byteorder='big')
            data += (startaddress).to_bytes(2, byteorder='big')
            data += (cnt).to_bytes(2, byteorder='big')
            return modbus_send_make(data)

        def modbus_fc06(address, value):
            global g_slaveid
            data = (g_slaveid).to_bytes(1, byteorder='big')
            data += (6).to_bytes(1, byteorder='big')
            data += (address).to_bytes(2, byteorder='big')
            data += (value).to_bytes(2, byteorder='big')
            return modbus_send_make(data)

        def modbus_fc16(startaddress, cnt, valuelist):
            global g_slaveid
            data = (g_slaveid).to_bytes(1, byteorder='big')
            data += (16).to_bytes(1, byteorder='big')
            data += (startaddress).to_bytes(2, byteorder='big')
            data += (cnt).to_bytes(2, byteorder='big')
            data += (2 * cnt).to_bytes(1, byteorder='big')
            for i in range(0, cnt):
                data += (valuelist[i]).to_bytes(2, byteorder='big')
            return modbus_send_make(data)

        def u32_to_words(value):
            low_word = value & 0xFFFF
            high_word = (value >> 16) & 0xFFFF
            return [low_word, high_word]

        def words_to_i32(low_word, high_word):
            value = low_word + (high_word << 16)
            if value >= 2147483648:
                value = value - 4294967296
            return value

        def recv_modbus_response(timeout, expected_length=0):
            deadline_ms = int(timeout * 1000)
            elapsed_ms = 0
            buffer = b""
            while elapsed_ms <= deadline_ms:
                size, val = flange_serial_read(0.05)
                if size > 0 and val is not None:
                    buffer += val
                    if expected_length > 0 and len(buffer) >= expected_length:
                        return True, buffer[0:expected_length]
                    if len(buffer) >= 3:
                        function_code = buffer[1]
                        if function_code == 3:
                            frame_length = 5 + buffer[2]
                            if len(buffer) >= frame_length:
                                return True, buffer[0:frame_length]
                        elif function_code == 6 or function_code == 16:
                            if len(buffer) >= 8:
                                return True, buffer[0:8]
                wait(0.01)
                elapsed_ms = elapsed_ms + 60
            return False, buffer

        def tcp_read_exact(size):
            global g_sock
            data = b""
            while len(data) < size:
                res, chunk = server_socket_read(g_sock, size - len(data), 1.0)
                if res < 0:
                    return res, None
                data += chunk
            return len(data), data

        def send_response(command, seq, payload):
            global g_sock
            tx_data = b"GP"
            tx_data += (1).to_bytes(1, byteorder='big')
            tx_data += (command).to_bytes(1, byteorder='big')
            tx_data += (seq).to_bytes(2, byteorder='big')
            tx_data += (len(payload)).to_bytes(2, byteorder='big')
            tx_data += payload
            server_socket_write(g_sock, tx_data)

        def open_server_socket():
            global g_sock
            while True:
                if g_sock is not None:
                    try:
                        server_socket_close(g_sock)
                    except:
                        pass
                    g_sock = None
                try:
                    g_sock = server_socket_open({port})
                    return
                except:
                    wait(0.5)

        def encode_state(status, moving, moving_status, present_current, present_temperature, present_velocity, present_position):
            global g_ready
            torque_flag = 1 if g_ready else 0
            payload = (status).to_bytes(1, byteorder='big')
            payload += (moving & 0xFF).to_bytes(1, byteorder='big')
            payload += (moving_status & 0xFF).to_bytes(1, byteorder='big')
            payload += (torque_flag).to_bytes(1, byteorder='big')
            payload += (present_current & 0xFFFF).to_bytes(2, byteorder='big')
            payload += (present_temperature & 0xFFFF).to_bytes(2, byteorder='big')
            payload += (present_velocity & 0xFFFFFFFF).to_bytes(4, byteorder='big')
            payload += (present_position & 0xFFFFFFFF).to_bytes(4, byteorder='big')
            return payload

        def read_state():
            try:
                flange_serial_write(modbus_fc03(ADDR_MOVING_STATUS, 13))
                ok, val = recv_modbus_response(0.5)
            except:
                return False, 0, 0, 0, 0, 0, 0
            if ok is False or val is None or len(val) < 31:
                return False, 0, 0, 0, 0, 0, 0

            moving = val[4]
            moving_status = val[3]
            present_current = int.from_bytes(val[7:9], byteorder='big', signed=True)
            velocity_low = int.from_bytes(val[9:11], byteorder='big', signed=False)
            velocity_high = int.from_bytes(val[11:13], byteorder='big', signed=False)
            present_velocity = words_to_i32(velocity_low, velocity_high)
            position_low = int.from_bytes(val[13:15], byteorder='big', signed=False)
            position_high = int.from_bytes(val[15:17], byteorder='big', signed=False)
            present_position = words_to_i32(position_low, position_high)
            present_temperature = val[28]
            return True, moving, moving_status, present_current, present_temperature, present_velocity, present_position

        def apply_profile():
            global g_last_accel, g_last_velocity
            try:
                flange_serial_write(modbus_fc06(ADDR_GOAL_CURRENT, g_goal_current))
                ok, val = recv_modbus_response(0.3, 8)
                if ok is False:
                    return False
                if g_profile_acceleration != g_last_accel:
                    flange_serial_write(modbus_fc16(ADDR_PROFILE_ACCELERATION, 2, u32_to_words(g_profile_acceleration)))
                    ok, val = recv_modbus_response(0.3, 8)
                    if ok is False:
                        return False
                    g_last_accel = g_profile_acceleration
                if g_profile_velocity != g_last_velocity:
                    flange_serial_write(modbus_fc16(ADDR_PROFILE_VELOCITY, 2, u32_to_words(g_profile_velocity)))
                    ok, val = recv_modbus_response(0.3, 8)
                    if ok is False:
                        return False
                    g_last_velocity = g_profile_velocity
            except:
                return False
            return True

        def open_serial():
            flange_serial_open(
                baudrate={baudrate},
                bytesize=DR_EIGHTBITS,
                parity=DR_PARITY_NONE,
                stopbits=DR_STOPBITS_ONE,
            )
            modbus_set_slaveid({slave_id})

        def reset_serial():
            flange_serial_close()
            wait(0.3)
            open_serial()

        def gripper_off():
            global g_ready
            g_ready = False
            try:
                flange_serial_write(modbus_fc06(ADDR_TORQUE_ENABLE, 0))
                recv_modbus_response(0.2, 8)
            except:
                pass
            try:
                flange_serial_close()
            except:
                pass

        def gripper_init():
            global g_ready
            g_ready = False
            try:
                flange_serial_close()
            except:
                pass
            wait(0.3)
            try:
                open_serial()
            except:
                gripper_off()
                return False

            attempts = 0
            ok = False
            while attempts < 10:
                try:
                    flange_serial_write(modbus_fc06(ADDR_TORQUE_ENABLE, 1))
                    res, val = recv_modbus_response(1.0, 8)
                except:
                    res = False
                if res is True:
                    ok = True
                    break
                attempts = attempts + 1
                if attempts == 5:
                    try:
                        reset_serial()
                    except:
                        wait(0.5)
                else:
                    wait(0.5)

            if ok is False:
                gripper_off()
                return False

            if apply_profile() is False:
                gripper_off()
                return False

            g_ready = True
            return True

        def wait_until_arrived(goal_position, timeout_ms):
            elapsed_ms = 0
            while True:
                ok, moving, moving_status, present_current, present_temperature, present_velocity, present_position = read_state()
                if ok is False:
                    return STATUS_IO_ERROR, moving, moving_status, present_current, present_temperature, present_velocity, present_position
                in_position = (moving_status & 0x01) == 1
                is_grasping = (abs(present_current) >= (g_goal_current * 0.9))
                if moving == 0 and (in_position or abs(goal_position - present_position) <= POSITION_TOLERANCE or is_grasping):
                    return STATUS_OK, moving, moving_status, present_current, present_temperature, present_velocity, present_position
                if timeout_ms > 0 and elapsed_ms >= timeout_ms:
                    return STATUS_TIMEOUT, moving, moving_status, present_current, present_temperature, present_velocity, present_position
                wait(POLL_WAIT_SEC)
                elapsed_ms = elapsed_ms + int(POLL_WAIT_SEC * 1000)

        def handle_initialize(command, seq, payload):
            global g_goal_current, g_ready
            if len(payload) >= 2:
                g_goal_current = int.from_bytes(payload[0:2], byteorder='big', signed=False)
            if g_ready is False:
                ok = gripper_init()
            else:
                ok = apply_profile()
                g_ready = ok
            if ok is False:
                send_response(command, seq, encode_state(STATUS_IO_ERROR, 0, 0, 0, 0, 0, 0))
                return
            state_ok, moving, moving_status, present_current, present_temperature, present_velocity, present_position = read_state()
            status = STATUS_OK if state_ok else STATUS_IO_ERROR
            send_response(command, seq, encode_state(status, moving, moving_status, present_current, present_temperature, present_velocity, present_position))

        def handle_set_torque(command, seq, payload):
            global g_ready
            if len(payload) != 2:
                send_response(command, seq, encode_state(STATUS_BAD_PACKET, 0, 0, 0, 0, 0, 0))
                return
            enable = int.from_bytes(payload[0:2], byteorder='big', signed=False) != 0
            if enable:
                try:
                    flange_serial_write(modbus_fc06(ADDR_TORQUE_ENABLE, 1))
                    ok, val = recv_modbus_response(0.3, 8)
                except:
                    ok = False
                if ok is False:
                    send_response(command, seq, encode_state(STATUS_IO_ERROR, 0, 0, 0, 0, 0, 0))
                    return
                g_ready = apply_profile()
            else:
                try:
                    flange_serial_write(modbus_fc06(ADDR_TORQUE_ENABLE, 0))
                    ok, val = recv_modbus_response(0.3, 8)
                except:
                    pass
                g_ready = False
            state_ok, moving, moving_status, present_current, present_temperature, present_velocity, present_position = read_state()
            status = STATUS_OK if state_ok else STATUS_IO_ERROR
            send_response(command, seq, encode_state(status, moving, moving_status, present_current, present_temperature, present_velocity, present_position))

        def handle_move(command, seq, payload):
            global g_goal_current, g_profile_velocity, g_profile_acceleration, g_last_accel, g_last_velocity
            if g_ready is False:
                send_response(command, seq, encode_state(STATUS_NOT_READY, 0, 0, 0, 0, 0, 0))
                return
            if len(payload) != 16:
                send_response(command, seq, encode_state(STATUS_BAD_PACKET, 0, 0, 0, 0, 0, 0))
                return

            pos_mm_x10  = int.from_bytes(payload[0:2],  byteorder='big', signed=False)
            force_ma    = int.from_bytes(payload[2:4],  byteorder='big', signed=False)
            vel_raw     = int.from_bytes(payload[4:8],  byteorder='big', signed=False)
            accel_raw   = int.from_bytes(payload[8:12], byteorder='big', signed=False)
            timeout_ms  = int.from_bytes(payload[12:16],byteorder='big', signed=False)

            if pos_mm_x10 < 0 or pos_mm_x10 > 1060:
                send_response(command, seq, encode_state(STATUS_RANGE_ERROR, 0, 0, 0, 0, 0, 0))
                return

            goal_position = mm_x10_to_raw(pos_mm_x10)
            g_goal_current = force_ma
            if vel_raw != g_profile_velocity:
                g_profile_velocity = vel_raw
                g_last_velocity = -1
            if accel_raw != g_profile_acceleration:
                g_profile_acceleration = accel_raw
                g_last_accel = -1

            if apply_profile() is False:
                send_response(command, seq, encode_state(STATUS_IO_ERROR, 0, 0, 0, 0, 0, 0))
                return
            try:
                flange_serial_write(modbus_fc16(ADDR_GOAL_POSITION, 2, u32_to_words(goal_position)))
                ok, val = recv_modbus_response(0.3, 8)
            except:
                ok = False
            if ok is False:
                send_response(command, seq, encode_state(STATUS_IO_ERROR, 0, 0, 0, 0, 0, 0))
                return
            if timeout_ms == 0:
                state_ok, moving, moving_status, present_current, present_temperature, present_velocity, present_position = read_state()
                status = STATUS_OK if state_ok else STATUS_IO_ERROR
                send_response(command, seq, encode_state(status, moving, moving_status, present_current, present_temperature, present_velocity, present_position))
                return
            status, moving, moving_status, present_current, present_temperature, present_velocity, present_position = wait_until_arrived(goal_position, timeout_ms)
            send_response(command, seq, encode_state(status, moving, moving_status, present_current, present_temperature, present_velocity, present_position))

        def handle_read(command, seq):
            if g_ready is False:
                send_response(command, seq, encode_state(STATUS_NOT_READY, 0, 0, 0, 0, 0, 0))
                return
            ok, moving, moving_status, present_current, present_temperature, present_velocity, present_position = read_state()
            status = STATUS_OK if ok else STATUS_IO_ERROR
            send_response(command, seq, encode_state(status, moving, moving_status, present_current, present_temperature, present_velocity, present_position))

        open_server_socket()

        while True:
            res, header = tcp_read_exact(HEADER_SIZE)
            if res < 0:
                open_server_socket()
                continue
            if header[0:2] != b"GP":
                continue
            version = header[2]
            command = header[3]
            seq = int.from_bytes(header[4:6], byteorder='big', signed=False)
            payload_size = int.from_bytes(header[6:8], byteorder='big', signed=False)
            if version != 1:
                send_response(command, seq, encode_state(STATUS_BAD_PACKET, 0, 0, 0, 0, 0, 0))
                continue
            payload = b""
            if payload_size > 0:
                res, payload = tcp_read_exact(payload_size)
                if res < 0:
                    open_server_socket()
                    continue
            if command == CMD_PING:
                handle_read(command, seq)
            elif command == CMD_INITIALIZE:
                handle_initialize(command, seq, payload)
            elif command == CMD_MOVE:
                handle_move(command, seq, payload)
            elif command == CMD_READ_STATE:
                handle_read(command, seq)
            elif command == CMD_SET_TORQUE:
                handle_set_torque(command, seq, payload)
            elif command == CMD_SHUTDOWN:
                handle_read(command, seq)
                break
            else:
                send_response(command, seq, encode_state(STATUS_BAD_COMMAND, 0, 0, 0, 0, 0, 0))

        if g_sock is not None:
            server_socket_close(g_sock)
        gripper_off()
    """).strip()
