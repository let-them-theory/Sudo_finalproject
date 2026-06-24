#!/usr/bin/env python3
"""E0509 pick-place scene: robot at home + table + cube (stable view)."""

from __future__ import annotations

import os


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

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="E0509 Isaac Sim scene (home pose)")
parser.add_argument("--task", type=str, default="Isaac-Lift-Cube-E0509-Play-v0")
parser.add_argument("--num_envs", type=int, default=1)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym
import isaac_e0509_pick_place  # noqa: F401
import torch
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

from isaac_e0509_pick_place.robots.home_reset import reset_robot_to_home

print("[INFO] Building E0509 scene (home + table + cube)...", flush=True)
env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=args.num_envs)
env = gym.make(args.task, cfg=env_cfg)
env.reset()
reset_robot_to_home(env.unwrapped.scene["robot"])

env.unwrapped.sim.set_camera_view(eye=(2.2, 1.6, 1.4), target=(0.45, 0.0, 0.45))
print("[OK] Robot at home [0,0,90,0,90,0] deg. Close window to exit.", flush=True)

device = env.unwrapped.device
while simulation_app.is_running():
    action = torch.zeros(env.action_space.shape, device=device)
    action[..., -1] = 1.0  # gripper open at home
    env.step(action)

env.close()
simulation_app.close()
print("[DONE]", flush=True)
