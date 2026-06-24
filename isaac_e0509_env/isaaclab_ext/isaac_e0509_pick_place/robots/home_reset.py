"""Restore E0509 articulation to the pick_place home pose in simulation."""

from __future__ import annotations

from typing import Sequence

import torch

from isaaclab.assets import Articulation

from .home_pose import home_joint_pos_rad

# reset_joints_by_scale multiplier for exact home (no randomization).
EXACT_HOME_JOINT_RANGE: tuple[float, float] = (1.0, 1.0)


def reset_robot_to_home(
    robot: Articulation,
    env_ids: Sequence[int] | torch.Tensor | None = None,
) -> None:
    """Write home joint state and PD targets (matches pick_place_params home_joints)."""
    if env_ids is not None and not isinstance(env_ids, torch.Tensor):
        env_ids = torch.tensor(list(env_ids), device=robot.device, dtype=torch.long)

    joint_pos = robot.data.default_joint_pos.clone()
    joint_vel = torch.zeros_like(joint_pos)
    name_to_idx = {name: idx for idx, name in enumerate(robot.data.joint_names)}
    for joint_name, value in home_joint_pos_rad().items():
        if joint_name in name_to_idx:
            joint_pos[:, name_to_idx[joint_name]] = value

    # env_ids=None → all environments (PhysX rejects slice).
    robot.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)
    robot.set_joint_position_target(joint_pos, env_ids=env_ids)
    robot.set_joint_velocity_target(joint_vel, env_ids=env_ids)
