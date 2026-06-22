#!/usr/bin/env python3
"""Train E0509 Lift with weights warm-started from a Reach checkpoint."""

import argparse
import os
import sys
from datetime import datetime

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Train E0509 lift with reach warm-start.")
parser.add_argument("--task", type=str, default="Isaac-Lift-Cube-E0509-v0")
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--max_iterations", type=int, default=1500)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument(
    "--reach_checkpoint",
    type=str,
    default=os.path.expanduser(
        "~/IsaacLab/logs/rsl_rl/e0509_reach/2026-06-16_10-34-11/model_999.pt"
    ),
    help="Reach policy checkpoint used to initialize lift actor/critic.",
)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym
import isaac_e0509_pick_place  # noqa: F401
import torch
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.utils.io import dump_yaml
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg
from isaaclab.utils.assets import retrieve_file_path
from rsl_rl.runners import OnPolicyRunner

from isaac_e0509_pick_place.tasks.lift.agents.rsl_rl_ppo_cfg import E0509LiftCubePPORunnerCfg
from isaac_e0509_pick_place.utils.policy_transfer import load_reach_into_lift_runner

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True
torch.backends.cudnn.deterministic = False
torch.backends.cudnn.benchmark = False


def main():
    agent_cfg = E0509LiftCubePPORunnerCfg()
    agent_cfg.seed = args.seed
    agent_cfg.max_iterations = args.max_iterations
    agent_cfg.run_name = "v2_clip"

    env_cfg: ManagerBasedRLEnvCfg = parse_env_cfg(args.task, device=args.device, num_envs=args.num_envs)
    env_cfg.seed = agent_cfg.seed

    log_root = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
    log_dir = datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + f"_{agent_cfg.run_name}"
    log_dir = os.path.join(log_root, log_dir)
    env_cfg.log_dir = log_dir
    print(f"[INFO] Logging to: {log_dir}", flush=True)

    env = gym.make(args.task, cfg=env_cfg, render_mode=None)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
    reach_ckpt = retrieve_file_path(args.reach_checkpoint)
    load_reach_into_lift_runner(runner, reach_ckpt, device=agent_cfg.device)

    os.makedirs(os.path.join(log_dir, "params"), exist_ok=True)
    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)
    dump_yaml(
        os.path.join(log_dir, "params", "reach_init.yaml"),
        {"reach_checkpoint": reach_ckpt},
    )

    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
