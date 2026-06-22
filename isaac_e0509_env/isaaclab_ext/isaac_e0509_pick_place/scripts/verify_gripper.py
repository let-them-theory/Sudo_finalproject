#!/usr/bin/env python3
"""Verify E0509 gripper opens/closes (all mimic joints driven)."""

from isaaclab.app import AppLauncher

import argparse

parser = argparse.ArgumentParser(description="E0509 gripper open/close check")
parser.add_argument("--task", type=str, default="Isaac-Lift-Cube-E0509-Play-v0")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym
import isaac_e0509_pick_place  # noqa: F401
import torch
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

from isaac_e0509_pick_place.robots.home_pose import GRIPPER_JOINTS

env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
env = gym.make(args.task, cfg=env_cfg)
env.reset()
robot = env.unwrapped.scene["robot"]


def gripper_pos() -> dict[str, float]:
    names = robot.data.joint_names
    pos = robot.data.joint_pos[0]
    return {n: pos[names.index(n)].item() for n in GRIPPER_JOINTS if n in names}


print("[OPEN] start", gripper_pos(), flush=True)
action = torch.zeros(env.action_space.shape, device=env.unwrapped.device)
action[..., -1] = 1.0  # binary open
for _ in range(80):
    env.step(action)
print("[OPEN] after +1", gripper_pos(), flush=True)

action[..., -1] = -1.0  # binary close
for _ in range(80):
    env.step(action)
print("[CLOSE] after -1", gripper_pos(), flush=True)

env.close()
simulation_app.close()
print("[OK] gripper check done", flush=True)
