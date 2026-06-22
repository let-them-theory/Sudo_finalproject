#!/usr/bin/env python3
"""EE-to-cube distance during reach-cube play."""

from isaaclab.app import AppLauncher
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="Isaac-Reach-Cube-E0509-Play-v0")
parser.add_argument("--checkpoint", type=str, default="/home/user/IsaacLab/logs/rsl_rl/e0509_reach/2026-06-16_10-34-11/model_999.pt")
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
from isaac_e0509_pick_place.tasks.reach.agents.rsl_rl_ppo_cfg import E0509ReachPPORunnerCfg

cfg = E0509ReachPPORunnerCfg()
env = gym.make(args.task, cfg=parse_env_cfg(args.task, device=args.device, num_envs=1))
env = RslRlVecEnvWrapper(env, clip_actions=cfg.clip_actions)
runner = OnPolicyRunner(env, cfg.to_dict(), log_dir=None, device=cfg.device)
runner.load(retrieve_file_path(args.checkpoint))
policy = runner.get_inference_policy(device=env.unwrapped.device)

obs = env.get_observations()
env.reset()
obs = env.get_observations()
cmd = env.unwrapped.command_manager.get_term("ee_pose")

for t in range(200):
    obs, _, _, _ = env.step(policy(obs))
    if t % 40 == 0:
        err = cmd.metrics["position_error"][0].item()
        raw = policy(obs)
        print(f"t={t} ee_goal_err={err:.3f}m action_max={raw.abs().max().item():.2f}", flush=True)

env.close()
simulation_app.close()
