#!/usr/bin/env python3
"""E0509 pick-place scene (robot + table + ground) via Isaac Lab Play env."""

from __future__ import annotations

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="E0509 Isaac Sim scene")
parser.add_argument("--task", type=str, default="Isaac-Reach-E0509-Play-v0")
parser.add_argument("--num_envs", type=int, default=1)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym
import isaac_e0509_pick_place  # noqa: F401  # register envs
import torch
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

print("[INFO] Building E0509 scene...", flush=True)
env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=args.num_envs)
# Keep exact home joints (no reset randomization).
env_cfg.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)

env = gym.make(args.task, cfg=env_cfg)
env.reset()

# Camera toward table / robot workspace.
env.unwrapped.sim.set_camera_view(eye=(2.2, 1.6, 1.4), target=(0.45, 0.0, 0.45))
print("[OK] E0509 scene ready (robot at home, table, ground). Close window to exit.", flush=True)

while simulation_app.is_running():
    action = torch.zeros(env.action_space.shape, device=env.unwrapped.device)
    env.step(action)

env.close()
simulation_app.close()
print("[DONE]", flush=True)
