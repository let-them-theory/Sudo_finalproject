#!/usr/bin/env python3
"""Sweep preset EE orientations: approach -> grasp -> close -> lift on dex cube."""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

# IK command frame is link_6 (flange). Waypoints are planned in TCP (fingertip) then converted.
# Offsets match pick_place_params.yaml / pick_place_node (ROS real robot).
DEX_CUBE_HALF_HEIGHT_M = 0.02   # 0.8-scaled dex cube (measured rest root z=0.02)
GRIPPER_CLOSE_LEN_M = 0.132   # flange→fingertip when closed (pick_place_node 실측)
PRE_PICK_Z_OFFSET_M = 0.06    # TCP above cube top — approach / lift (kept in arm workspace)
PICK_Z_OFFSET_M = -0.02       # TCP relative to cube top at grasp (negative = fingertips at cube center)
MIN_SAFE_TCP_Z_M = 0.36       # allow aiming into the cube body (DLS undershoots ~5 cm)

LIFT_SUCCESS_Z_M = 0.47   # cube starts ~0.42 (table top 0.40); success = raised ~0.05
SETTLE_STEPS = 60
OBJECT_SETTLE_STEPS = 40
EE_BODY = "link_6"
FINGER_TIP_BODIES = ("gripper_rh_p12_rn_r2", "gripper_rh_p12_rn_l2")
# Nominal spawn (robot base frame). Sync with e0509_lift_env_cfg.OBJECT_INIT_POS.
OBJECT_POS_B = (0.30, 0.0, 0.42)


def _cube_top_z(object_root_z: float) -> float:
    return object_root_z + DEX_CUBE_HALF_HEIGHT_M


def _object_pose_base(env) -> tuple[torch.Tensor, torch.Tensor]:
    """Object root pose in robot base frame (from live sim buffers)."""
    obj = env.unwrapped.scene["object"]
    robot = _robot(env)
    return math_utils.subtract_frame_transforms(
        robot.data.root_pos_w,
        robot.data.root_quat_w,
        obj.data.root_pos_w,
        obj.data.root_quat_w,
    )


def _fingertip_dir_in_base(env) -> torch.Tensor:
    """Unit vector link_6 → fingertips (from live finger body positions)."""
    robot = _robot(env)
    link_pos_b, _ = _ee_pose_base(env)
    tips_b: list[torch.Tensor] = []
    for body_name in FINGER_TIP_BODIES:
        body_ids, _ = robot.find_bodies(body_name)
        if not body_ids:
            continue
        tip_w = robot.data.body_pos_w[:, body_ids[0]]
        tip_b, _ = math_utils.subtract_frame_transforms(
            robot.data.root_pos_w,
            robot.data.root_quat_w,
            tip_w,
            robot.data.body_quat_w[:, body_ids[0]],
        )
        tips_b.append(tip_b[0])
    if tips_b:
        mid = torch.stack(tips_b).mean(dim=0)
        vec = mid - link_pos_b[0]
        n = vec.norm()
        if n.item() > 1e-4:
            return vec / n
    # Fallback: fingers hang in base -Z at home (Isaac tool axis is flipped vs mesh).
    return torch.tensor([0.0, 0.0, -1.0], device=link_pos_b.device, dtype=torch.float32)


def _measure_fingertip_extent(env) -> float:
    """Distance link_6 → lowest fingertip along finger direction (open gripper)."""
    robot = _robot(env)
    link_pos_b, _ = _ee_pose_base(env)
    finger_dir = _fingertip_dir_in_base(env)
    max_extent = 0.0
    found = False
    for body_name in FINGER_TIP_BODIES:
        body_ids, _ = robot.find_bodies(body_name)
        if not body_ids:
            continue
        tip_w = robot.data.body_pos_w[:, body_ids[0]]
        tip_b, _ = math_utils.subtract_frame_transforms(
            robot.data.root_pos_w,
            robot.data.root_quat_w,
            tip_w,
            robot.data.body_quat_w[:, body_ids[0]],
        )
        extent = torch.dot(tip_b[0] - link_pos_b[0], finger_dir).item()
        max_extent = max(max_extent, extent)
        found = True
    if not found or max_extent < 0.01:
        return GRIPPER_CLOSE_LEN_M
    return max_extent


def _link6_from_tcp(env, tcp_b: torch.Tensor, extent_m: float) -> torch.Tensor:
    """link_6 pose so fingertips (extent_m along finger dir) sit at tcp_b."""
    return tcp_b - _fingertip_dir_in_base(env) * extent_m


PRE_BACKOFF_M = 0.10   # pre-grasp backs off this far along the approach axis
LIFT_HEIGHT_M = 0.10   # lift straight up after grasp
GRASP_SINK_M = 0.06    # aim grasp below cube center to counter IK undershoot


def _compute_grasp_waypoints(
    env,
    obj_pos_b: torch.Tensor,
    approach_dir: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, dict[str, float]]:
    """link_6 IK goals: fingertips at cube center, approaching along ``approach_dir``.

    link_6 sits one finger-extent back along ``approach_dir`` so the fingertips land
    on the cube; pre-grasp backs off further along the same axis; lift goes straight up.
    """
    extent_open = _measure_fingertip_extent(env)
    extent_close = max(extent_open, GRIPPER_CLOSE_LEN_M)

    grasp_tcp = obj_pos_b.clone()  # fingertips at cube center
    # Aim deeper to compensate IK undershoot (~3 cm), clamped just above the table.
    grasp_tcp[2] = max(obj_pos_b[2].item() - GRASP_SINK_M, MIN_SAFE_TCP_Z_M + 0.005)
    grasp_b = grasp_tcp - approach_dir * extent_close
    pre_b = grasp_b - approach_dir * PRE_BACKOFF_M
    lift_b = grasp_b.clone()
    lift_b[2] += LIFT_HEIGHT_M

    meta = {
        "cube_top": _cube_top_z(obj_pos_b[2].item()),
        "extent_open": extent_open,
        "extent_close": extent_close,
        "grasp_tcp_z": grasp_tcp[2].item(),
        "pre_tcp_z": pre_b[2].item(),
        "lift_tcp_z": lift_b[2].item(),
    }
    return pre_b, grasp_b, lift_b, meta


def _strip_stale_colcon_overlay() -> None:
    paths = [
        p for p in os.environ.get("PYTHONPATH", "").split(":")
        if p and "/install/isaac_e0509_pick_place/" not in p
    ]
    if paths:
        os.environ["PYTHONPATH"] = ":".join(paths)
    else:
        os.environ.pop("PYTHONPATH", None)


_strip_stale_colcon_overlay()

from isaaclab.app import AppLauncher  # noqa: E402

import argparse  # noqa: E402

parser = argparse.ArgumentParser(description="E0509 multi-angle grasp motion sweep")
parser.add_argument("--task", type=str, default="Isaac-Lift-Cube-E0509-Grasp-Play-v0")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument(
    "--preset",
    type=str,
    default="top_down",
    help="Preset name, or 'all' for full sweep (default: top_down)",
)
parser.add_argument("--object-yaw-deg", type=float, default=0.0, help="Rotate cube on table (deg)")
parser.add_argument("--steps-pre", type=int, default=160)
parser.add_argument("--steps-grasp", type=int, default=120)
parser.add_argument("--steps-close", type=int, default=100)
parser.add_argument("--steps-lift", type=int, default=160)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import isaac_e0509_pick_place  # noqa: F401  # register envs
import isaaclab.utils.math as math_utils  # noqa: E402
import torch  # noqa: E402
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg  # noqa: E402

from isaac_e0509_pick_place.robots.home_reset import reset_robot_to_home


@dataclass(frozen=True)
class GraspPreset:
    """Delta orientation (rad) applied on top of home EE pose in robot base frame."""

    name: str
    delta_roll: float
    delta_pitch: float
    delta_yaw: float


# Deltas relative to home EE — avoids wrong absolute euler → wild IK twists.
GRASP_PRESETS: tuple[GraspPreset, ...] = (
    GraspPreset("top_down", 0.0, 0.0, 0.0),
    GraspPreset("tilt_150", 0.0, math.radians(-30.0), 0.0),
    GraspPreset("tilt_135", 0.0, math.radians(-45.0), 0.0),
    GraspPreset("yaw_45", 0.0, 0.0, math.radians(45.0)),
    GraspPreset("yaw_90", 0.0, 0.0, math.radians(90.0)),
    GraspPreset("roll_30", math.radians(30.0), 0.0, 0.0),
)


def _robot(env):
    return env.unwrapped.scene["robot"]


def _physics_flush(env, steps: int = 2) -> None:
    """Advance sim so articulation/object buffers match written state."""
    scene = env.unwrapped.scene
    dt = env.unwrapped.physics_dt
    for _ in range(steps):
        scene.write_data_to_sim()
        env.unwrapped.sim.step()
        scene.update(dt)


def _ee_body_index(env) -> int:
    body_ids, _ = _robot(env).find_bodies(EE_BODY)
    return body_ids[0]


def _ee_pose_base(env) -> tuple[torch.Tensor, torch.Tensor]:
    """End-effector pose in robot base frame (matches IK absolute command frame)."""
    robot = _robot(env)
    body_idx = _ee_body_index(env)
    ee_pos_w = robot.data.body_pos_w[:, body_idx]
    ee_quat_w = robot.data.body_quat_w[:, body_idx]
    return math_utils.subtract_frame_transforms(
        robot.data.root_pos_w,
        robot.data.root_quat_w,
        ee_pos_w,
        ee_quat_w,
    )


def _delta_quat(roll: float, pitch: float, yaw: float, device: str) -> torch.Tensor:
    return math_utils.quat_from_euler_xyz(
        torch.tensor([roll], device=device),
        torch.tensor([pitch], device=device),
        torch.tensor([yaw], device=device),
    )[0]


def _align_quat_base(env, home_quat_b: torch.Tensor, target_dir: torch.Tensor) -> torch.Tensor:
    """EE orientation whose finger direction points along ``target_dir`` (base frame).

    Minimal rotation aligning the live finger direction with ``target_dir``, applied
    on top of home. Used to aim the gripper from a reachable spot toward the cube,
    so link_6 stays in the arm's workspace (a pure top-down aim is out of reach).
    """
    fd = _fingertip_dir_in_base(env)
    tgt = target_dir / target_dir.norm()
    axis = torch.cross(fd, tgt, dim=0)
    axis_n = axis.norm()
    if axis_n.item() < 1e-6:
        return home_quat_b
    axis = axis / axis_n
    angle = torch.acos(torch.clamp(torch.dot(fd, tgt), -1.0, 1.0))
    r_align = math_utils.quat_from_angle_axis(angle.unsqueeze(0), axis.unsqueeze(0))[0]
    return math_utils.quat_mul(r_align.unsqueeze(0), home_quat_b.unsqueeze(0))[0]


def _target_quat_base(base_quat_b: torch.Tensor, preset: GraspPreset, device: str) -> torch.Tensor:
    """Preset delta applied on top of the true top-down orientation."""
    delta = _delta_quat(preset.delta_roll, preset.delta_pitch, preset.delta_yaw, device)
    return math_utils.quat_mul(delta.unsqueeze(0), base_quat_b.unsqueeze(0))[0]


def _make_action(
    device: str,
    pos_b: torch.Tensor | tuple[float, float, float],
    quat_b: torch.Tensor,
    gripper: float,
) -> torch.Tensor:
    if not isinstance(pos_b, torch.Tensor):
        pos_b = torch.tensor(pos_b, device=device, dtype=torch.float32)
    return torch.tensor(
        [[pos_b[0], pos_b[1], pos_b[2], quat_b[0], quat_b[1], quat_b[2], quat_b[3], gripper]],
        device=device,
        dtype=torch.float32,
    )


def _hold_pose(env, pos_b: torch.Tensor, quat_b: torch.Tensor, gripper: float, steps: int) -> None:
    action = _make_action(env.unwrapped.device, pos_b, quat_b, gripper)
    for _ in range(steps):
        env.step(action)


def _move_linear(
    env,
    start_b: torch.Tensor,
    end_b: torch.Tensor,
    quat_b: torch.Tensor,
    gripper: float,
    steps: int,
) -> None:
    device = env.unwrapped.device
    for i in range(steps):
        alpha = float(i + 1) / float(steps)
        pos_b = start_b * (1.0 - alpha) + end_b * alpha
        env.step(_make_action(device, pos_b, quat_b, gripper))


def _approach_via_safe_transit(
    env,
    start_b: torch.Tensor,
    goal_b: torch.Tensor,
    quat_b: torch.Tensor,
    gripper: float,
    steps: int,
    clearance_m: float = 0.15,
) -> None:
    """Lift → move XY → descend (avoids sweeping through the cube)."""
    safe_z = max(start_b[2].item(), goal_b[2].item()) + clearance_m
    lift_b = start_b.clone()
    lift_b[2] = safe_z
    transit_b = goal_b.clone()
    transit_b[2] = safe_z
    third = max(steps // 3, 1)
    _move_linear(env, start_b, lift_b, quat_b, gripper, third)
    _move_linear(env, lift_b, transit_b, quat_b, gripper, third)
    _move_linear(env, transit_b, goal_b, quat_b, gripper, max(steps - 2 * third, 1))


def _object_z(env) -> float:
    pos_b, _ = _object_pose_base(env)
    return pos_b[0, 2].item()


def _fingertip_mid_base(env) -> torch.Tensor:
    """Actual midpoint of the two fingertips in robot base frame (live)."""
    robot = _robot(env)
    tips_b: list[torch.Tensor] = []
    for body_name in FINGER_TIP_BODIES:
        body_ids, _ = robot.find_bodies(body_name)
        if not body_ids:
            continue
        tip_w = robot.data.body_pos_w[:, body_ids[0]]
        tip_b, _ = math_utils.subtract_frame_transforms(
            robot.data.root_pos_w,
            robot.data.root_quat_w,
            tip_w,
            robot.data.body_quat_w[:, body_ids[0]],
        )
        tips_b.append(tip_b[0])
    return torch.stack(tips_b).mean(dim=0)


def _servo_horizontal_over_cube(
    env,
    quat_b: torch.Tensor,
    iters: int = 7,
    steps: int = 20,
    tol: float = 0.010,
) -> torch.Tensor:
    """Closed-loop X/Y alignment of the fingertip midpoint over the cube.

    Z is held fixed (above the cube) so the open fingers do not contact and shove
    the cube while aligning. DLS undershoots each command, so we iterate.
    """
    pos = _ee_pose_base(env)[0][0]
    for k in range(iters):
        cube = _object_pos_b(env)
        tip = _fingertip_mid_base(env)
        err = cube - tip
        err[2] = 0.0  # horizontal only — stay above the cube
        n = err.norm().item()
        pos = _ee_pose_base(env)[0][0]
        print(
            f"    servoXY[{k}] gapXY={n:.3f} tip=({tip[0]:.3f},{tip[1]:.3f},{tip[2]:.3f})"
            f" cube=({cube[0]:.3f},{cube[1]:.3f},{cube[2]:.3f})",
            flush=True,
        )
        if n < tol:
            break
        _move_linear(env, pos, pos + err, quat_b, 1.0, steps)
    return _ee_pose_base(env)[0][0]


def _plunge_onto_cube(
    env,
    quat_b: torch.Tensor,
    sink_below_center: float = 0.01,
    xy_gain: float = 0.6,
    z_step: float = 0.015,
    max_iters: int = 22,
    steps_per: int = 8,
) -> torch.Tensor:
    """Closed-loop descent: lower link_6 while re-aiming X/Y over the cube each step.

    A straight vertical plunge drifts forward (DLS z-x coupling) and misses the cube,
    so we interleave small z drops with X/Y corrections (gain < 1 for stability) until
    the fingertips reach just below the cube center, straddling it.
    """
    target_z = None
    pos = _ee_pose_base(env)[0][0]
    for k in range(max_iters):
        cube = _object_pos_b(env)
        tip = _fingertip_mid_base(env)
        pos = _ee_pose_base(env)[0][0]
        if target_z is None:
            target_z = cube[2].item() - sink_below_center
        err = cube - tip
        dz_remaining = tip[2].item() - target_z
        target = pos.clone()
        target[0] = pos[0] + xy_gain * err[0]
        target[1] = pos[1] + xy_gain * err[1]
        target[2] = pos[2] - min(max(dz_remaining, 0.0), z_step)
        print(
            f"    plunge[{k}] tipz={tip[2].item():.3f}->{target_z:.3f}"
            f" gapXY={err[:2].norm().item():.3f}",
            flush=True,
        )
        if dz_remaining < 0.008 and err[:2].norm().item() < 0.012:
            break
        _move_linear(env, pos, target, quat_b, 1.0, steps_per)
    return _ee_pose_base(env)[0][0]


def _log_grasp_geometry(env, tag: str) -> None:
    """Actual fingertip vs cube gap — reveals if the EE truly reaches the cube."""
    tip = _fingertip_mid_base(env)
    ee_pos, _ = _ee_pose_base(env)
    cube = _object_pos_b(env)
    gap = (tip - cube).norm().item()
    print(
        f"  [{tag}] fingertip_b=({tip[0]:.3f},{tip[1]:.3f},{tip[2]:.3f})"
        f" ee_b=({ee_pos[0,0]:.3f},{ee_pos[0,1]:.3f},{ee_pos[0,2]:.3f})"
        f" cube_b=({cube[0]:.3f},{cube[1]:.3f},{cube[2]:.3f}) gap={gap:.3f}",
        flush=True,
    )


def _park_object_out_of_way(env) -> None:
    """Move cube away while the arm settles (avoids home motion knocking it off the table)."""
    obj = env.unwrapped.scene["object"]
    device = env.unwrapped.device
    origin = env.unwrapped.scene.env_origins[0]
    root = obj.data.default_root_state[0].clone()
    root[:3] = origin + torch.tensor([0.0, 0.0, 1.5], device=device, dtype=torch.float32)
    root[7:] = 0.0
    obj.write_root_state_to_sim(
        root.unsqueeze(0),
        env_ids=torch.tensor([0], device=device, dtype=torch.long),
    )
    _physics_flush(env, steps=2)


def _spawn_object_on_table(env, yaw_rad: float = 0.0) -> None:
    """Force cube onto table at OBJECT_INIT_POS (fixes fall-through / knock-off)."""
    obj = env.unwrapped.scene["object"]
    device = env.unwrapped.device
    origin = env.unwrapped.scene.env_origins[0]

    pos_w = torch.tensor(OBJECT_POS_B, device=device, dtype=torch.float32) + origin
    quat = math_utils.quat_from_euler_xyz(
        torch.zeros(1, device=device),
        torch.zeros(1, device=device),
        torch.tensor([yaw_rad], device=device),
    )[0]

    root = obj.data.default_root_state[0].clone()
    root[:3] = pos_w
    root[3:7] = quat
    root[7:] = 0.0
    obj.write_root_state_to_sim(
        root.unsqueeze(0),
        env_ids=torch.tensor([0], device=device, dtype=torch.long),
    )
    _physics_flush(env, steps=2)


def _object_pos_b(env) -> torch.Tensor:
    pos_b, _ = _object_pose_base(env)
    return pos_b[0].clone()


def _ensure_cube_on_table(
    env,
    object_yaw: float,
    hold_pos_b: torch.Tensor,
    hold_quat_b: torch.Tensor,
    settle_steps: int = 0,
) -> torch.Tensor:
    """Re-seat cube and optionally hold arm pose while physics settles."""
    _spawn_object_on_table(env, object_yaw)
    if settle_steps > 0:
        _hold_pose(env, hold_pos_b, hold_quat_b, 1.0, settle_steps)
    return _object_pos_b(env)


def _prepare_episode(env, object_yaw: float) -> tuple[torch.Tensor, torch.Tensor]:
    """Reset robot, settle at home, place cube on table."""
    env.reset()
    _park_object_out_of_way(env)
    reset_robot_to_home(_robot(env))
    _physics_flush(env, steps=3)
    home_pos_b, home_quat_b = _ee_pose_base(env)
    _hold_pose(env, home_pos_b[0], home_quat_b[0], 1.0, SETTLE_STEPS)

    print("  arm settled at home (cube parked until grasp sequence)", flush=True)
    return home_pos_b, home_quat_b


def _resolve_presets() -> tuple[GraspPreset, ...]:
    if args.preset == "all":
        return GRASP_PRESETS
    match = tuple(p for p in GRASP_PRESETS if p.name == args.preset)
    if not match:
        names = ", ".join(p.name for p in GRASP_PRESETS)
        raise ValueError(f"Unknown preset {args.preset!r}. Choose from: {names}, all")
    return match


def main() -> None:
    env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=args.num_envs)
    # Scripted motion exceeds default 8s episode; disable drop timeout during debug.
    env_cfg.episode_length_s = 600.0
    env_cfg.terminations.object_dropping.params["minimum_height"] = -0.25

    env = gym.make(args.task, cfg=env_cfg)
    device = env.unwrapped.device
    presets = _resolve_presets()

    env.unwrapped.sim.set_camera_view(eye=(2.0, 1.5, 1.35), target=(0.45, 0.0, 0.48))
    object_yaw = math.radians(args.object_yaw_deg)

    results: list[tuple[str, float, bool]] = []
    for preset in presets:
        print(f"\n=== preset: {preset.name} "
              f"(delta rpy deg={math.degrees(preset.delta_roll):.1f}, "
              f"{math.degrees(preset.delta_pitch):.1f}, {math.degrees(preset.delta_yaw):.1f}) ===",
              flush=True)

        home_pos_b, home_quat_b = _prepare_episode(env, object_yaw)

        # Home now points the fingers down/forward (grasp-ready); use it directly as
        # the base orientation and apply only the preset delta. top_down (delta 0)
        # means no reorientation, so the gripper never sweeps through the cube.
        base_quat_b = home_quat_b[0]
        target_quat_b = _target_quat_base(base_quat_b, preset, device)
        _fd = _fingertip_dir_in_base(env)
        print(
            f"  home ee_b=({home_pos_b[0,0]:.3f},{home_pos_b[0,1]:.3f},{home_pos_b[0,2]:.3f})"
            f" finger_dir=({_fd[0]:.3f},{_fd[1]:.3f},{_fd[2]:.3f})",
            flush=True,
        )

        # Apply preset delta (open gripper) and settle.
        cur_pos_b, _ = _ee_pose_base(env)
        _hold_pose(env, cur_pos_b[0], target_quat_b, 1.0, 40)
        _fd2 = _fingertip_dir_in_base(env)
        print(f"  finger_dir after delta=({_fd2[0]:.3f},{_fd2[1]:.3f},{_fd2[2]:.3f})", flush=True)

        # Lift the open gripper clear of the cube spawn area, THEN spawn + settle the
        # cube (home fingers overlap the spawn point and would eject the cube).
        cur_pos_b, _ = _ee_pose_base(env)
        clear_b = cur_pos_b[0].clone()
        clear_b[2] += 0.15
        _move_linear(env, cur_pos_b[0], clear_b, target_quat_b, 1.0, 40)
        obj_pos_b = _ensure_cube_on_table(env, object_yaw, clear_b, target_quat_b, settle_steps=40)
        pre_b, grasp_b, lift_b, wpt = _compute_grasp_waypoints(env, obj_pos_b, _fd2)

        print(
            f"  cube base ({obj_pos_b[0]:.3f}, {obj_pos_b[1]:.3f}, {obj_pos_b[2]:.3f})"
            f"  top={wpt['cube_top']:.3f}",
            flush=True,
        )
        print(
            f"  TCP z: pre={wpt['pre_tcp_z']:.3f} grasp={wpt['grasp_tcp_z']:.3f}"
            f" lift={wpt['lift_tcp_z']:.3f}"
            f"  finger_extent open={wpt['extent_open']:.3f} close={wpt['extent_close']:.3f}",
            flush=True,
        )
        print(
            f"  link_6 z: pre={pre_b[2].item():.3f} grasp={grasp_b[2].item():.3f}"
            f" lift={lift_b[2].item():.3f}",
            flush=True,
        )

        # Already reoriented above; move to pre-grasp (above cube) → descend.
        cur_pos_b, _ = _ee_pose_base(env)
        print(f"  cube on table z={obj_pos_b[2].item():.3f} — pre, grasp", flush=True)
        _move_linear(env, cur_pos_b[0], pre_b, target_quat_b, 1.0, args.steps_pre)
        print(f"  after pre-grasp  object_z={_object_z(env):.3f}", flush=True)
        _log_grasp_geometry(env, "pre-grasp")

        # pre already aligns X/Y over the cube by construction; a horizontal servo
        # here diverges (DLS is unstable at this extended config). Plunge directly.
        # Plunge straight down so the open fingers straddle the cube.
        grasp_pos_b = _plunge_onto_cube(env, target_quat_b)
        print(f"  after plunge     object_z={_object_z(env):.3f}", flush=True)
        _log_grasp_geometry(env, "plunge")

        # Close at depth (hold position so the arm doesn't drift back while shutting).
        _hold_pose(env, grasp_pos_b, target_quat_b, -1.0, args.steps_close)
        print(f"  after close      object_z={_object_z(env):.3f}", flush=True)
        _log_grasp_geometry(env, "close")

        lift_target = grasp_pos_b.clone()
        lift_target[2] += LIFT_HEIGHT_M
        cur_pos_b, _ = _ee_pose_base(env)
        _move_linear(env, cur_pos_b[0], lift_target, target_quat_b, -1.0, args.steps_lift)
        final_z = _object_z(env)
        success = final_z >= LIFT_SUCCESS_Z_M
        results.append((preset.name, final_z, success))
        print(f"  after lift       object_z={final_z:.3f}  success={success}", flush=True)

        if not args.headless and len(presets) == 1:
            print("[INFO] Restoring home pose — close window to exit.", flush=True)
            reset_robot_to_home(_robot(env))
            _spawn_object_on_table(env, object_yaw)
            home_pos_b, home_quat_b = _ee_pose_base(env)
            for _ in range(SETTLE_STEPS):
                env.step(_make_action(device, home_pos_b[0], home_quat_b[0], 1.0))
            while simulation_app.is_running():
                home_pos_b, home_quat_b = _ee_pose_base(env)
                env.step(_make_action(device, home_pos_b[0], home_quat_b[0], 1.0))
            break  # user closed window — skip summary duplicate close

    print("\n=== summary ===", flush=True)
    for name, z, ok in results:
        print(f"  {name:12s}  final_z={z:.3f}  lifted={ok}", flush=True)

    env.close()
    simulation_app.close()
    print("[DONE] grasp motion sweep finished", flush=True)


if __name__ == "__main__":
    main()
