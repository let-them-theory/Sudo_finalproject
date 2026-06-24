"""E0509-specific MDP reward terms (dense grasp shaping)."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

# Cache fingertip body / gripper joint indices per robot (find_* does regex matching).
_FINGER_IDS: dict[int, list[int]] = {}
_GRIP_JOINT_ID: dict[int, int] = {}


def _finger_ids(robot: Articulation, finger_body_names: tuple[str, ...]) -> list[int]:
    key = id(robot)
    ids = _FINGER_IDS.get(key)
    if ids is None:
        ids = []
        for name in finger_body_names:
            found, _ = robot.find_bodies(name)
            if found:
                ids.append(found[0])
        _FINGER_IDS[key] = ids
    return ids


def fingertips_object_distance(
    env: ManagerBasedRLEnv,
    std: float,
    finger_body_names: tuple[str, ...] = ("gripper_rh_p12_rn_r2", "gripper_rh_p12_rn_l2"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Dense grasp shaping: reward both fingertips hugging the object.

    Mean over fingers of a tanh kernel on the fingertip->object distance. High only
    when BOTH fingertips are near the object (i.e. the cube is between the fingers),
    which the sparse lift reward alone never bootstraps for the RH-P12 gripper.
    """
    obj: RigidObject = env.scene[object_cfg.name]
    robot: Articulation = env.scene[robot_cfg.name]
    obj_pos = obj.data.root_pos_w  # (N, 3)
    ids = _finger_ids(robot, finger_body_names)
    if not ids:
        return torch.zeros(env.num_envs, device=env.device)
    rew = torch.zeros(env.num_envs, device=env.device)
    for bid in ids:
        tip = robot.data.body_pos_w[:, bid]
        dist = torch.norm(tip - obj_pos, dim=1)
        rew += 1.0 - torch.tanh(dist / std)
    return rew / len(ids)


def grasp_closed_on_object(
    env: ManagerBasedRLEnv,
    std: float,
    gripper_joint: str = "gripper_rh_r1",
    open_rad: float = 1.0,
    finger_body_names: tuple[str, ...] = ("gripper_rh_p12_rn_r2", "gripper_rh_p12_rn_l2"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reward = (fingertips near object) x (gripper closed).

    Proximity alone lets the policy hover open fingers by the cube for free reward
    without ever grasping. Gating by gripper closedness (joint 0=closed, open_rad=open)
    means reward is high ONLY when the gripper closes around the object — the signal
    that bootstraps an actual grasp before the (sparse) lift reward can fire.
    """
    robot: Articulation = env.scene[robot_cfg.name]
    proximity = fingertips_object_distance(
        env, std, finger_body_names=finger_body_names, object_cfg=object_cfg, robot_cfg=robot_cfg
    )
    key = id(robot)
    jid = _GRIP_JOINT_ID.get(key)
    if jid is None:
        found, _ = robot.find_joints(gripper_joint)
        jid = found[0] if found else 0
        _GRIP_JOINT_ID[key] = jid
    joint = robot.data.joint_pos[:, jid]
    closedness = torch.clamp(1.0 - joint / open_rad, 0.0, 1.0)
    return proximity * closedness
