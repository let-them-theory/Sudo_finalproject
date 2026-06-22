#!/usr/bin/env python3
"""Diagnose play behavior: actions, smoothness, EE vs cube distance."""

from isaaclab.app import AppLauncher
import argparse, os
parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", type=str, default=os.path.expanduser(
    "~/IsaacLab/logs/rsl_rl/e0509_lift/2026-06-16_12-08-52_v2_clip/model_1499.pt"))
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

env = gym.make("Isaac-Lift-Cube-E0509-Play-v0", cfg=parse_env_cfg("Isaac-Lift-Cube-E0509-Play-v0", device=args.device, num_envs=1))
agent_cfg = E0509LiftCubePPORunnerCfg()
env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
runner.load(retrieve_file_path(args.checkpoint))
policy = runner.get_inference_policy(device=env.unwrapped.device)

obs = env.get_observations()
env.reset()
obs = env.get_observations()

prev_a = None
robot = env.unwrapped.scene["robot"]
obj = env.unwrapped.scene["object"]
ee = env.unwrapped.scene["ee_frame"]

for t in range(300):
    a = policy(obs)
    if prev_a is not None:
        da = (a - prev_a).abs().max().item()
    else:
        da = 0.0
    prev_a = a.clone()

    obs, rew, _, _ = env.step(a)

    if t % 30 == 0:
        ee_pos = ee.data.target_pos_w[0, 0]
        obj_pos = obj.data.root_pos_w[0]
        dist = torch.linalg.norm(ee_pos - obj_pos).item()
        arm = a[0, :6].tolist()
        print(
            f"t={t:3d} dist={dist:.3f}m rew={rew.item():.3f} "
            f"max|da|={da:.3f} arm={[round(x,2) for x in arm]} grip={a[0,6].item():.2f}",
            flush=True,
        )

env.close()
simulation_app.close()
