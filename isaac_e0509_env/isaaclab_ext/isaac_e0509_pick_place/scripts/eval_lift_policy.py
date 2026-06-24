#!/usr/bin/env python3
"""Measure lift success rate of a trained E0509 lift policy over randomized resets.

Loads an rsl_rl checkpoint, runs the (randomized) lift env across many envs, and
reports the fraction of envs whose object is lifted above a height threshold.
"""

from isaaclab.app import AppLauncher

import argparse

parser = argparse.ArgumentParser(description="E0509 lift policy success-rate eval")
parser.add_argument("--task", type=str, default="Isaac-Lift-Cube-E0509-v0")
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--episodes", type=int, default=4, help="reset cycles to average over")
parser.add_argument("--lift_thresh", type=float, default=0.46, help="object z (base) counted as lifted")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
import isaac_e0509_pick_place  # noqa: F401
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from rsl_rl.runners import OnPolicyRunner

env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=args.num_envs)
agent_cfg = parse_env_cfg  # placeholder; runner cfg loaded below

env = gym.make(args.task, cfg=env_cfg)
env = RslRlVecEnvWrapper(env)

# Build runner and load the policy weights.
from isaac_e0509_pick_place.tasks.lift.agents.rsl_rl_ppo_cfg import E0509LiftCubePPORunnerCfg

runner_cfg = E0509LiftCubePPORunnerCfg().to_dict()
runner = OnPolicyRunner(env, runner_cfg, log_dir=None, device=args.device)
runner.load(args.checkpoint)
policy = runner.get_inference_policy(device=args.device)

unwrapped = env.unwrapped
obj = unwrapped.scene["object"]
robot = unwrapped.scene["robot"]
import isaaclab.utils.math as math_utils


def object_z_base() -> torch.Tensor:
    pos_b, _ = math_utils.subtract_frame_transforms(
        robot.data.root_pos_w, robot.data.root_quat_w, obj.data.root_pos_w, obj.data.root_quat_w
    )
    return pos_b[:, 2]


steps_per_ep = int(unwrapped.max_episode_length)
successes = 0
total = 0
obs, _ = env.reset()
for ep in range(args.episodes):
    peak_z = torch.full((args.num_envs,), -10.0, device=args.device)
    for _ in range(steps_per_ep):
        with torch.inference_mode():
            actions = policy(obs)
        obs, _, _, _ = env.step(actions)
        peak_z = torch.maximum(peak_z, object_z_base())
    lifted = (peak_z >= args.lift_thresh).sum().item()
    successes += lifted
    total += args.num_envs
    print(f"  episode {ep}: lifted {lifted}/{args.num_envs} (peak z mean={peak_z.mean().item():.3f})", flush=True)
    obs, _ = env.reset()

print(f"\n[RESULT] lift success rate = {successes}/{total} = {100.0*successes/total:.1f}%"
      f" (threshold z>={args.lift_thresh})", flush=True)
env.close()
simulation_app.close()
