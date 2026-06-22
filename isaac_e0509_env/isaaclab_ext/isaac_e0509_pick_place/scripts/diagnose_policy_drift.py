#!/usr/bin/env python3
"""Compare zero-action hold vs trained lift policy on joint stability."""

from isaaclab.app import AppLauncher

import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="Isaac-Lift-Cube-E0509-Play-v0")
parser.add_argument(
    "--checkpoint",
    type=str,
    default=os.path.expanduser(
        "~/IsaacLab/logs/rsl_rl/e0509_lift/2026-06-16_12-08-52_v2_clip/model_1499.pt"
    ),
)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym
import isaac_e0509_pick_place  # noqa: F401
import torch
from isaaclab.utils.assets import retrieve_file_path
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg
from rsl_rl.runners import OnPolicyRunner

from isaac_e0509_pick_place.tasks.lift.agents.rsl_rl_ppo_cfg import E0509LiftCubePPORunnerCfg


def max_joint_drift(env) -> float:
    robot = env.unwrapped.scene["robot"]
    pos = robot.data.joint_pos[0, :6]
    default = robot.data.default_joint_pos[0, :6]
    return (pos - default).abs().max().item()


def run_steps(env, policy_fn, steps: int, label: str) -> None:
    env.reset()
    obs = env.get_observations()
    for t in range(steps):
        obs, _, _, _ = env.step(policy_fn(obs))
        if t in (0, 9, 49, 99, 199):
            print(f"{label} t={t+1} max_drift={max_joint_drift(env):.4f}", flush=True)


env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
env = gym.make(args.task, cfg=env_cfg)
env = RslRlVecEnvWrapper(env, clip_actions=1.0)

run_steps(env, lambda obs: torch.zeros(env.action_space.shape, device=env.unwrapped.device), 200, "[zero]")

agent_cfg = E0509LiftCubePPORunnerCfg()
runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
runner.load(retrieve_file_path(args.checkpoint))
policy = runner.get_inference_policy(device=env.unwrapped.device)
run_steps(env, policy, 200, "[policy]")

env.close()
simulation_app.close()
