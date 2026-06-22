"""Gym registration for E0509 lift (pick warmup) tasks."""

import gymnasium as gym

from . import agents
from .e0509_lift_env_cfg import E0509CubeLiftEnvCfg, E0509CubeLiftEnvCfg_PLAY

gym.register(
    id="Isaac-Lift-Cube-E0509-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.e0509_lift_env_cfg:E0509CubeLiftEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:E0509LiftCubePPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Lift-Cube-E0509-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.e0509_lift_env_cfg:E0509CubeLiftEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:E0509LiftCubePPORunnerCfg",
    },
)
