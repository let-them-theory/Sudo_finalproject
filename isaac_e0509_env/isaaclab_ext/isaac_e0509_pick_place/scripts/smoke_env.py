#!/usr/bin/env python3
"""Smoke test: spawn E0509+gripper in Isaac Lab reach env."""

from isaaclab.app import AppLauncher

import argparse

parser = argparse.ArgumentParser(description="E0509 Isaac Lab smoke test")
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

env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=args.num_envs)
env = gym.make(args.task, cfg=env_cfg)
env.reset()
print(f"[OK] env ready: {env}")
for step in range(200):
    action = 2.0 * torch.rand(env.action_space.shape, device=env.unwrapped.device) - 1.0
    obs, rew, terminated, truncated, info = env.step(action)
    if step % 50 == 0:
        print(f"step={step} reward_mean={rew.mean().item():.4f}")
env.close()
simulation_app.close()
print("[DONE] smoke test finished", flush=True)
