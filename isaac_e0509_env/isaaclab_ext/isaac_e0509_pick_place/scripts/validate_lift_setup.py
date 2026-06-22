#!/usr/bin/env python3
"""Preflight: home pose hold + clipped-policy sanity before play/train."""

from isaaclab.app import AppLauncher

import argparse
import os
import sys

parser = argparse.ArgumentParser(description="Validate E0509 lift physics and policy")
parser.add_argument("--task", type=str, default="Isaac-Lift-Cube-E0509-Play-v0")
parser.add_argument(
    "--checkpoint",
    type=str,
    default="",
    help="Optional checkpoint; if set, also validate clipped policy drift.",
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

from isaac_e0509_pick_place.robots.home_pose import home_joint_pos_rad
from isaac_e0509_pick_place.tasks.lift.agents.rsl_rl_ppo_cfg import E0509LiftCubePPORunnerCfg

HOME_DRIFT_MAX = 0.06
POLICY_DRIFT_MAX = 0.45


def max_arm_drift(env) -> float:
    robot = env.unwrapped.scene["robot"]
    names = robot.data.joint_names
    expected = home_joint_pos_rad()
    drifts = []
    for i in range(1, 7):
        n = f"joint_{i}"
        idx = names.index(n)
        drifts.append(abs(robot.data.joint_pos[0, idx].item() - expected[n]))
    return max(drifts)


def hold_zero(env, steps: int) -> float:
    env.reset()
    zero = torch.zeros(env.action_space.shape, device=env.unwrapped.device)
    worst = 0.0
    for _ in range(steps):
        env.step(zero)
        worst = max(worst, max_arm_drift(env))
    return worst


def main() -> int:
    agent_cfg = E0509LiftCubePPORunnerCfg()
    env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
    env = gym.make(args.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    zero_drift = hold_zero(env, 120)
    print(f"[CHECK] zero-action home drift: {zero_drift:.4f} (max {HOME_DRIFT_MAX})", flush=True)
    ok = zero_drift <= HOME_DRIFT_MAX

    if args.checkpoint:
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
        runner.load(retrieve_file_path(os.path.expanduser(args.checkpoint)))
        policy = runner.get_inference_policy(device=env.unwrapped.device)
        env.reset()
        obs = env.get_observations()
        worst = 0.0
        for _ in range(150):
            obs, _, _, _ = env.step(policy(obs))
            worst = max(worst, max_arm_drift(env))
        print(f"[CHECK] clipped-policy drift: {worst:.4f} (max {POLICY_DRIFT_MAX})", flush=True)
        ok = ok and worst <= POLICY_DRIFT_MAX

    env.close()
    simulation_app.close()
    if ok:
        print("[PASS] lift setup OK", flush=True)
        return 0
    print("[FAIL] lift setup out of tolerance — retrain or fix robot cfg", flush=True)
    return 1


if __name__ == "__main__":
    sys.exit(main())
