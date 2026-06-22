#!/usr/bin/env python3
"""Headless numeric eval: trained policy vs random actions on E0509 Reach."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Evaluate E0509 reach policy numerically")
parser.add_argument("--task", type=str, default="Isaac-Reach-E0509-v0")
parser.add_argument("--num_envs", type=int, default=16)
parser.add_argument("--steps", type=int, default=400)
parser.add_argument(
    "--checkpoint",
    type=str,
    default="~/IsaacLab/logs/rsl_rl/e0509_reach/2026-06-16_10-34-11/model_999.pt",
)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym
import isaac_e0509_pick_place  # noqa: F401
import torch
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from isaaclab.utils.assets import retrieve_file_path
from rsl_rl.runners import OnPolicyRunner

from isaac_e0509_pick_place.tasks.reach.agents.rsl_rl_ppo_cfg import E0509ReachPPORunnerCfg


def run_episode(env, policy_fn, steps: int) -> dict[str, float]:
  pos_errors: list[float] = []
  ori_errors: list[float] = []
  rewards: list[float] = []

  obs = env.get_observations()
  for _ in range(steps):
    actions = policy_fn(obs)
    obs, rew, _, _ = env.step(actions)
    cmd = env.unwrapped.command_manager.get_term("ee_pose")
    pos_errors.append(cmd.metrics["position_error"].mean().item())
    ori_errors.append(cmd.metrics["orientation_error"].mean().item())
    rewards.append(rew.mean().item())

  return {
    "position_error_m": sum(pos_errors) / len(pos_errors),
    "orientation_error": sum(ori_errors) / len(ori_errors),
    "mean_reward": sum(rewards) / len(rewards),
  }


def main():
  env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=args.num_envs)
  env = gym.make(args.task, cfg=env_cfg)
  env = RslRlVecEnvWrapper(env)

  random_stats = run_episode(
    env,
    lambda obs: 2.0 * torch.rand(env.action_space.shape, device=env.unwrapped.device) - 1.0,
    args.steps,
  )
  print("[RANDOM]", random_stats, flush=True)

  agent_cfg = E0509ReachPPORunnerCfg()
  runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
  ckpt = retrieve_file_path(args.checkpoint)
  runner.load(ckpt)
  policy = runner.get_inference_policy(device=env.unwrapped.device)

  trained_stats = run_episode(env, policy, args.steps)
  print("[TRAINED]", trained_stats, flush=True)

  pos_improve = (random_stats["position_error_m"] - trained_stats["position_error_m"]) / random_stats[
    "position_error_m"
  ] * 100.0
  rew_improve = trained_stats["mean_reward"] - random_stats["mean_reward"]
  print(
    f"[VERDICT] position_error improved {pos_improve:.1f}% | reward delta {rew_improve:+.3f}",
    flush=True,
  )

  env.close()
  simulation_app.close()


if __name__ == "__main__":
  main()
