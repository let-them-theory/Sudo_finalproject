#!/usr/bin/env python3
"""Smoke test: E0509 + dex cube lift env."""

from __future__ import annotations

import os


def _strip_stale_colcon_overlay() -> None:
    """ROS colcon install can shadow the editable Isaac Lab extension."""
    paths = [
        p for p in os.environ.get("PYTHONPATH", "").split(":")
        if p and "/install/isaac_e0509_pick_place/" not in p
    ]
    if paths:
        os.environ["PYTHONPATH"] = ":".join(paths)
    else:
        os.environ.pop("PYTHONPATH", None)


_strip_stale_colcon_overlay()

from isaaclab.app import AppLauncher

import argparse

parser = argparse.ArgumentParser(description="E0509 lift env smoke test")
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

env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=args.num_envs)
env = gym.make(args.task, cfg=env_cfg)
env.reset()
print(f"[OK] lift env ready: {env}", flush=True)
for step in range(200):
    action = 2.0 * torch.rand(env.action_space.shape, device=env.unwrapped.device) - 1.0
    obs, rew, terminated, truncated, info = env.step(action)
    if step % 50 == 0:
        obj_z = env.unwrapped.scene["object"].data.root_pos_w[0, 2].item()
        print(f"step={step} reward_mean={rew.mean().item():.4f} object_z={obj_z:.3f}", flush=True)
env.close()
simulation_app.close()
print("[DONE] lift smoke test finished", flush=True)
