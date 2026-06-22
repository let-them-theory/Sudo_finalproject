#!/usr/bin/env python3
"""Print policy actions vs joint targets for lift play."""

from isaaclab.app import AppLauncher
import argparse, os
parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", type=str, default=os.path.expanduser("~/IsaacLab/logs/rsl_rl/e0509_lift/2026-06-16_11-44-37_from_reach/model_1499.pt"))
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
env = RslRlVecEnvWrapper(env)
runner = OnPolicyRunner(env, E0509LiftCubePPORunnerCfg().to_dict(), log_dir=None, device=args.device)
runner.load(retrieve_file_path(args.checkpoint))
policy = runner.get_inference_policy(device=args.device)

obs = env.get_observations()
env.reset()
obs = env.get_observations()
for t in range(10):
    act = policy(obs)
    print(f"t={t} actions={[round(x,3) for x in act[0].tolist()]}", flush=True)
    obs, _, _, _ = env.step(act)

robot = env.unwrapped.scene["robot"]
names = robot.data.joint_names
print("joint positions:", flush=True)
for n in names:
    i = names.index(n)
    print(f"  {n}: pos={robot.data.joint_pos[0,i].item():.3f} default={robot.data.default_joint_pos[0,i].item():.3f}", flush=True)

env.close()
simulation_app.close()
