#!/usr/bin/env python3
"""Sweep candidate arm configs; report EE pos + finger direction in base frame.

Goal: find joint angles where the gripper points down/forward (toward the cube at
~(0.45,0,0.52)) so a grasp needs only a small reorientation. Prints, per config,
the link_6 position and the link_6->fingertip unit vector (want ~(*, *, -1)).
"""

from isaaclab.app import AppLauncher

import argparse

parser = argparse.ArgumentParser(description="E0509 home-pose finder")
parser.add_argument("--task", type=str, default="Isaac-Lift-Cube-E0509-Grasp-Play-v0")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import itertools
import math

import gymnasium as gym
import isaac_e0509_pick_place  # noqa: F401
import isaaclab.utils.math as math_utils
import torch
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

EE_BODY = "link_6"
FINGER_TIP_BODIES = ("gripper_rh_p12_rn_r2", "gripper_rh_p12_rn_l2")
CUBE_B = torch.tensor([0.45, 0.0, 0.52])

env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
env = gym.make(args.task, cfg=env_cfg)
env.reset()
robot = env.unwrapped.scene["robot"]
device = env.unwrapped.device
name_to_idx = {n: i for i, n in enumerate(robot.data.joint_names)}


def set_arm(deg6):
    pos = robot.data.joint_pos.clone()
    for i, d in enumerate(deg6, start=1):
        pos[:, name_to_idx[f"joint_{i}"]] = math.radians(d)
    for n in ("gripper_rh_r1", "gripper_rh_r2", "gripper_rh_l1", "gripper_rh_l2"):
        if n in name_to_idx:
            pos[:, name_to_idx[n]] = 1.0  # open
    vel = torch.zeros_like(pos)
    robot.write_joint_state_to_sim(pos, vel)
    robot.set_joint_position_target(pos)
    dt = env.unwrapped.physics_dt
    for _ in range(8):
        env.unwrapped.scene.write_data_to_sim()
        env.unwrapped.sim.step()
        env.unwrapped.scene.update(dt)


def ee_and_finger():
    bid = robot.find_bodies(EE_BODY)[0][0]
    ee_w = robot.data.body_pos_w[:, bid]
    ee_b, _ = math_utils.subtract_frame_transforms(
        robot.data.root_pos_w, robot.data.root_quat_w, ee_w, robot.data.body_quat_w[:, bid]
    )
    tips = []
    for nm in FINGER_TIP_BODIES:
        tb = robot.find_bodies(nm)[0]
        if not tb:
            continue
        tw = robot.data.body_pos_w[:, tb[0]]
        tbf, _ = math_utils.subtract_frame_transforms(
            robot.data.root_pos_w, robot.data.root_quat_w, tw, robot.data.body_quat_w[:, tb[0]]
        )
        tips.append(tbf[0])
    mid = torch.stack(tips).mean(dim=0)
    fd = mid - ee_b[0]
    fd = fd / fd.norm()
    return ee_b[0], fd


# Sweep wrist (joint_4/5/6) for downward finger dir; vary shoulder/elbow for reach.
candidates = []
for j2, j3 in ((0, 90), (20, 70), (35, 70), (45, 60)):
    for j4 in (0, 90, -90, 180):
        for j5 in (-90, -45, 0, 45, 90):
            for j6 in (0, 90):
                candidates.append((0, j2, j3, j4, j5, j6))

print(f"[INFO] testing {len(candidates)} configs (want finger_dir z ~ -1, ee in front)", flush=True)
best = []
for c in candidates:
    set_arm(c)
    ee, fd = ee_and_finger()
    # score: prefer fingers down (fd_z negative), ee in front (+x) and reachable height
    in_front = ee[0].item() > 0.25
    # lower = better: fingers downward (fd_z negative), in front, ee height near 0.50
    score = fd[2].item() + (0.0 if in_front else 1.0) + 0.3 * abs(ee[2].item() - 0.50)
    best.append((score, c, ee.tolist(), fd.tolist()))

best.sort(key=lambda r: r[0])
print("\n=== top 12 (fingers most downward, in front) ===", flush=True)
for score, c, ee, fd in best[:12]:
    print(
        f"  cfg={c} ee=({ee[0]:.3f},{ee[1]:.3f},{ee[2]:.3f})"
        f" finger_dir=({fd[0]:.3f},{fd[1]:.3f},{fd[2]:.3f}) score={score:.3f}",
        flush=True,
    )

env.close()
simulation_app.close()
print("[DONE] home pose finder", flush=True)
