"""E0509 lift play env with differential IK (grasp motion verification)."""

import math

from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.utils import configclass

from isaac_e0509_pick_place.robots import E0509_GRIPPER_CFG
from isaac_e0509_pick_place.tasks.reach.e0509_reach_env_cfg import (
    EE_PITCH_RANGE,
    EE_ROLL_RANGE,
    EE_YAW_RANGE,
)

from .e0509_lift_env_cfg import E0509CubeLiftEnvCfg_PLAY

EE_BODY = "link_6"


@configclass
class E0509CubeLiftEnvCfg_GRASP_PLAY(E0509CubeLiftEnvCfg_PLAY):
    """Lift play scene with stiff E0509 drives + absolute-pose IK arm control."""

    def __post_init__(self):
        super().__post_init__()

        # Moderate PD for smoother IK tracking (8000/N scene cfg oscillates on E0509).
        _arm = E0509_GRIPPER_CFG.actuators["arm"].replace(stiffness=800.0, damping=80.0)
        _gripper = E0509_GRIPPER_CFG.actuators["gripper"].replace(stiffness=400.0, damping=40.0)
        self.scene.robot = E0509_GRIPPER_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot",
            actuators={"arm": _arm, "gripper": _gripper},
        )

        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["joint_[1-6]"],
            body_name=EE_BODY,
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=False,
                ik_method="dls",
            ),
        )

        # Scripted grasp sweep — keep cube fixed on reset.
        self.events.reset_object_position.params["pose_range"] = {
            "x": (0.0, 0.0),
            "y": (0.0, 0.0),
            "z": (0.0, 0.0),
            "yaw": (0.0, 0.0),
        }

        # Wider lift command ranges for multi-angle EE goals (debug markers only).
        self.commands.object_pose.ranges.roll = EE_ROLL_RANGE
        self.commands.object_pose.ranges.pitch = EE_PITCH_RANGE
        self.commands.object_pose.ranges.yaw = EE_YAW_RANGE
        self.commands.object_pose.resampling_time_range = (30.0, 30.0)
        self.events.reset_all.params["reset_joint_targets"] = True
