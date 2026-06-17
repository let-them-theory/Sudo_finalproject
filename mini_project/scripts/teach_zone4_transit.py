#!/usr/bin/env python3
"""zone4 transit 관절각 티치 보조 — 접근 높이 계산, 이동, 관절각 캡처.

사용 순서 (로봇·dsr01 드라이버 기동 후):

  1) 목표 높이 확인
     python3 teach_zone4_transit.py info

  2) zone4 접근 자세로 이동 (카테시안)
     python3 teach_zone4_transit.py goto

  3) 티치 팬던트에서 XY·Z·자세 미세 조정 (MANUAL 모드)

  4) 현재 관절각 읽기 → yaml 붙여넣기용 출력
     python3 teach_zone4_transit.py capture

  5) (선택) 캡처한 관절각으로 movej 시험
     python3 teach_zone4_transit.py test-movej

  6) (선택) capture 결과를 yaml zone4 열에 자동 반영
     python3 teach_zone4_transit.py capture --apply
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


ZONE4_IDX = 3
DEFAULT_NS = 'dsr01'
DEFAULT_CONFIG = (
    Path(__file__).resolve().parent.parent / 'config' / 'pick_place_params.yaml'
)
J_KEYS = [f'sort_roi_zone_place_j{i}' for i in range(1, 7)]


def _load_zone4_params(config_path: Path) -> dict:
    text = config_path.read_text(encoding='utf-8')
    block = text.split('pick_place_node:')[-1]

    def _arr(name: str) -> list[float]:
        m = re.search(rf'{name}:\s*\[([^\]]+)\]', block)
        if not m:
            raise KeyError(f'{name} not found in {config_path}')
        return [float(x.strip()) for x in m.group(1).split(',')]

    return {
        'px': _arr('sort_roi_zone_positions_x')[ZONE4_IDX],
        'py': _arr('sort_roi_zone_positions_y')[ZONE4_IDX],
        'pz': _arr('sort_roi_zone_positions_z')[ZONE4_IDX],
        'rx': _arr('sort_roi_zone_place_rx')[ZONE4_IDX],
        'ry': _arr('sort_roi_zone_place_ry')[ZONE4_IDX],
        'rz': _arr('sort_roi_zone_place_rz')[ZONE4_IDX],
        'approach_dz': float(
            re.search(r'place_approach_z_offset:\s*([-\d.]+)', block).group(1)),
        'joints': [_arr(k)[ZONE4_IDX] for k in J_KEYS],
        'joint_vel': float(re.search(r'joint_vel:\s*([-\d.]+)', block).group(1)),
        'joint_acc': float(re.search(r'joint_acc:\s*([-\d.]+)', block).group(1)),
        'cart_vel': float(re.search(r'cart_vel:\s*([-\d.]+)', block).group(1)),
        'cart_acc': float(re.search(r'cart_acc:\s*([-\d.]+)', block).group(1)),
    }


def _approach_z(p: dict) -> float:
    return p['pz'] + p['approach_dz']


def _print_info(p: dict, approach_offset: float | None) -> None:
    dz = approach_offset if approach_offset is not None else p['approach_dz']
    az = p['pz'] + dz
    print('=== zone4 transit 티치 목표 ===')
    print(f'  place_xy       : ({p["px"]:.3f}, {p["py"]:.3f}) m')
    print(f'  place_z        : {p["pz"]:.3f} m')
    print(f'  approach offset: {dz:.3f} m  (place_approach_z_offset)')
    print(f'  approach_z     : {az:.3f} m  ← transit 티치 TCP Z 목표')
    print(f'  place RPY      : rx={p["rx"]:.2f}, ry={p["ry"]:.2f}, rz={p["rz"]:.2f} deg')
    print(f'  현재 yaml 관절  : {p["joints"]}')
    print()
    print('절차:')
    print('  1. place_approach_z_offset 을 원하는 값으로 yaml 수정')
    print('  2. teach_zone4_transit.py goto  → 접근 자세로 이동')
    print('  3. 티치 팬던트 MANUAL에서 XY·Z·RPY 미세 조정')
    print('  4. teach_zone4_transit.py capture [--apply]')


def _format_yaml_joints(joints: list[float]) -> None:
    print('\n# pick_place_params.yaml — zone4 열(4번째 값)만 교체')
    for i, j in enumerate(joints, start=1):
        v = round(j, 1)
        print(f'    sort_roi_zone_place_j{i}: [..., ..., ..., {v:>6}, ...]')


def _apply_joints_to_yaml(config_path: Path, joints: list[float]) -> None:
    lines = config_path.read_text(encoding='utf-8').splitlines(keepends=True)
    out: list[str] = []
    for line in lines:
        updated = line
        for i, key in enumerate(J_KEYS, start=1):
            if line.strip().startswith(f'{key}:'):
                m = re.match(rf'(\s*{key}:\s*\[)([^\]]+)(\])', line)
                if not m:
                    raise RuntimeError(f'parse failed: {line!r}')
                vals = [x.strip() for x in m.group(2).split(',')]
                if len(vals) <= ZONE4_IDX:
                    raise RuntimeError(f'{key} needs index {ZONE4_IDX}')
                vals[ZONE4_IDX] = f'{round(joints[i - 1], 1):.1f}'
                updated = f'{m.group(1)}{", ".join(vals)}{m.group(3)}\n'
                break
        out.append(updated)
    config_path.write_text(''.join(out), encoding='utf-8')
    print(f'Applied zone4 joints to {config_path}')


class TeachZone4Node:
    def __init__(self, ns: str):
        import rclpy
        from rclpy.node import Node
        from dsr_msgs2.srv import GetCurrentPosj, GetCurrentPosx, MoveJoint, MoveLine

        self._rclpy = rclpy
        self._node = Node('teach_zone4_transit')
        self._GetCurrentPosj = GetCurrentPosj
        self._GetCurrentPosx = GetCurrentPosx
        self._MoveJoint = MoveJoint
        self._MoveLine = MoveLine
        prefix = f'/{ns}'
        self.cli_posj = self._node.create_client(GetCurrentPosj, f'{prefix}/motion/get_current_posj')
        self.cli_posx = self._node.create_client(GetCurrentPosx, f'{prefix}/motion/get_current_posx')
        self.cli_movej = self._node.create_client(MoveJoint, f'{prefix}/motion/move_joint')
        self.cli_movel = self._node.create_client(MoveLine, f'{prefix}/motion/move_line')

    def destroy_node(self):
        self._node.destroy_node()

    def _wait(self, client, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service unavailable: {client.srv_name}')
        return client

    def read_posj(self) -> list[float]:
        self._wait(self.cli_posj)
        fut = self.cli_posj.call_async(self._GetCurrentPosj.Request())
        self._rclpy.spin_until_future_complete(self._node, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None or not res.success:
            raise RuntimeError('get_current_posj failed')
        return [float(v) for v in res.pos]

    def read_posx_m(self) -> tuple[float, float, float, float, float, float]:
        self._wait(self.cli_posx)
        req = self._GetCurrentPosx.Request()
        req.ref = 0
        fut = self.cli_posx.call_async(req)
        self._rclpy.spin_until_future_complete(self._node, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None or not res.success or not res.task_pos_info:
            raise RuntimeError('get_current_posx failed')
        raw = [float(v) for v in res.task_pos_info[0].data]
        if len(raw) < 6:
            raise RuntimeError(f'unexpected posx length: {raw}')
        x, y, z, rx, ry, rz = raw[:6]
        return x / 1000.0, y / 1000.0, z / 1000.0, rx, ry, rz

    def move_line(self, x, y, z, rpy, vel, acc):
        self._wait(self.cli_movel)
        req = self._MoveLine.Request()
        req.pos = [x * 1000.0, y * 1000.0, z * 1000.0,
                   float(rpy[0]), float(rpy[1]), float(rpy[2])]
        req.vel = [vel, 30.0]
        req.acc = [acc, 60.0]
        req.time = 0.0
        req.radius = 0.0
        req.ref = 0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0
        fut = self.cli_movel.call_async(req)
        self._rclpy.spin_until_future_complete(self._node, fut, timeout_sec=60.0)
        if fut.result() is None:
            raise RuntimeError('move_line failed')

    def move_joint(self, joints, vel, acc):
        self._wait(self.cli_movej)
        req = self._MoveJoint.Request()
        req.pos = [float(v) for v in joints]
        req.vel = vel
        req.acc = acc
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0
        fut = self.cli_movej.call_async(req)
        self._rclpy.spin_until_future_complete(self._node, fut, timeout_sec=60.0)
        if fut.result() is None:
            raise RuntimeError('move_joint failed')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='zone4 transit joint teach helper')
    parser.add_argument(
        'command',
        choices=['info', 'goto', 'capture', 'test-movej'],
        help='info=목표 출력, goto=접근자세 이동, capture=관절각 읽기, test-movej=yaml 관절 시험',
    )
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--ns', default=DEFAULT_NS)
    parser.add_argument(
        '--approach-offset', type=float, default=None,
        help='place_approach_z_offset 임시 덮어쓰기(m). yaml 수정 전 시험용.',
    )
    parser.add_argument(
        '--apply', action='store_true',
        help='capture 시 zone4 관절각을 yaml에 자동 반영',
    )
    parser.add_argument(
        '--joints', type=float, nargs=6, default=None,
        help='test-movej용 관절각(deg). 미지정 시 yaml 값 사용.',
    )
    args = parser.parse_args(argv)

    params = _load_zone4_params(args.config)
    if args.command == 'info':
        _print_info(params, args.approach_offset)
        return 0

    import rclpy
    rclpy.init()
    node = TeachZone4Node(args.ns)
    try:
        dz = args.approach_offset if args.approach_offset is not None else params['approach_dz']
        az = params['pz'] + dz
        rpy = [params['rx'], params['ry'], params['rz']]

        if args.command == 'goto':
            print(f'Moving to zone4 approach: ({params["px"]:.3f}, {params["py"]:.3f}, {az:.3f})')
            print(f'RPY: {rpy}')
            node.move_line(params['px'], params['py'], az, rpy,
                           params['cart_vel'], params['cart_acc'])
            x, y, z, rx, ry, rz = node.read_posx_m()
            print(f'Arrived TCP: ({x:.3f}, {y:.3f}, {z:.3f}) m, RPY=({rx:.2f}, {ry:.2f}, {rz:.2f})')
            print(f'Target approach_z={az:.3f}, delta_z={z - az:+.3f} m')
            print('티치 팬던트 MANUAL에서 미세 조정 후 capture 실행하세요.')

        elif args.command == 'capture':
            joints = node.read_posj()
            x, y, z, rx, ry, rz = node.read_posx_m()
            print('=== captured ===')
            print(f'TCP (m) : x={x:.3f}, y={y:.3f}, z={z:.3f}')
            print(f'RPY (deg): rx={rx:.2f}, ry={ry:.2f}, rz={rz:.2f}')
            print(f'Joints  : {[round(j, 1) for j in joints]}')
            print(f'Target approach_z={az:.3f}, delta_z={z - az:+.3f} m')
            if abs(z - az) > 0.02:
                print('  ! Z 차이 > 2cm — place_approach_z_offset 또는 티치 높이를 다시 맞추세요.')
            _format_yaml_joints(joints)
            if args.apply:
                _apply_joints_to_yaml(args.config, joints)
                print('pick_place_node 재시작 후 test-movej 로 확인하세요.')

        elif args.command == 'test-movej':
            joints = list(args.joints) if args.joints else list(params['joints'])
            print(f'test movej: {joints}')
            node.move_joint(joints, params['joint_vel'], params['joint_acc'])
            x, y, z, rx, ry, rz = node.read_posx_m()
            print(f'After movej TCP: ({x:.3f}, {y:.3f}, {z:.3f}) m')
            print(f'Target approach_z={az:.3f}, delta_z={z - az:+.3f} m')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
