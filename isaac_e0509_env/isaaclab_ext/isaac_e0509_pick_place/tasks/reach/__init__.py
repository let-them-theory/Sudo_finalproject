"""Gym registration for E0509 reach tasks."""

import gymnasium as gym

from . import agents
from .e0509_reach_env_cfg import E0509ReachEnvCfg, E0509ReachEnvCfg_PLAY

gym.register(
    id="Isaac-Reach-E0509-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.e0509_reach_env_cfg:E0509ReachEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:E0509ReachPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Reach-E0509-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.e0509_reach_env_cfg:E0509ReachEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:E0509ReachPPORunnerCfg",
    },
)
