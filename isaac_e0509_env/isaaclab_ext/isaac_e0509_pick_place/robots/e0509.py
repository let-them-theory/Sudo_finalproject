"""Doosan E0509 with RH-P12 gripper articulation configuration."""

from __future__ import annotations

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from .home_pose import home_joint_pos_rad

# isaac_e0509_env/assets/urdf/e0509_gripper_isaac.urdf
_ENV_ROOT = Path(__file__).resolve().parents[3]
URDF_PATH = _ENV_ROOT / "assets/urdf/e0509_gripper_isaac.urdf"

E0509_GRIPPER_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=str(URDF_PATH),
        fix_base=True,
        merge_fixed_joints=True,
        make_instanceable=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=1,
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=None, damping=None)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos=home_joint_pos_rad(),
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint_[1-6]"],
            effort_limit_sim=120.0,
            stiffness=400.0,
            damping=40.0,
        ),
        "gripper": ImplicitActuatorCfg(
            # Drive all four finger joints directly: Isaac's URDF import drops the
            # <mimic> coupling, so r2/l1/l2 stay limp unless actuated. multiplier=1
            # in the URDF means identical targets reproduce the intended motion.
            joint_names_expr=["gripper_rh_r1", "gripper_rh_r2", "gripper_rh_l1", "gripper_rh_l2"],
            effort_limit_sim=50.0,
            stiffness=400.0,
            damping=40.0,
        ),
    },
)
