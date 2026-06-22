#!/usr/bin/env python3
"""Check E0509 home pose, actuator hold, and joint targets at reset."""

from isaaclab.app import AppLauncher

import argparse
import math

parser = argparse.ArgumentParser(description="Diagnose E0509 physics / home pose")
parser.add_argument("--task", type=str, default="Isaac-Lift-Cube-E0509-Play-v0")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym
import isaac_e0509_pick_place  # noqa: F401
import torch
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

from isaac_e0509_pick_place.robots.home_pose import HOME_JOINTS_DEG, home_joint_pos_rad

env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
env = gym.make(args.task, cfg=env_cfg)
env.reset()
robot = env.unwrapped.scene["robot"]
names = list(robot.data.joint_names)
expected = home_joint_pos_rad()

print("=== joint names ===", names, flush=True)
print("=== expected home (rad) ===", expected, flush=True)

def arm_state(label: str) -> None:
    pos = robot.data.joint_pos[0]
    vel = robot.data.joint_vel[0]
    for i in range(1, 7):
        n = f"joint_{i}"
        idx = names.index(n)
        exp = expected.get(n, float("nan"))
        print(
            f"{label} {n}: pos={pos[idx].item():.4f} (exp {exp:.4f}) vel={vel[idx].item():.4f}",
            flush=True,
        )


arm_state("[reset]")
zero = torch.zeros(env.action_space.shape, device=env.unwrapped.device)
for step in range(120):
    env.step(zero)
    if step in (0, 29, 59, 119):
        arm_state(f"[zero-action step {step+1}]")

env.close()
simulation_app.close()
