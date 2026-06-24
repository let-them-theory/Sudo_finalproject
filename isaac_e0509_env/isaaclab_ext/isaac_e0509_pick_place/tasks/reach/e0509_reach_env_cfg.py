"""E0509 reach task (approach / pre-pick warmup)."""

import math

from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.manipulation.reach.mdp as mdp
from isaaclab_tasks.manager_based.manipulation.reach.reach_env_cfg import ReachEnvCfg

from isaac_e0509_pick_place.robots import E0509_GRIPPER_CFG
from isaac_e0509_pick_place.robots.home_pose import GRIPPER_JOINTS
from isaac_e0509_pick_place.robots.home_reset import EXACT_HOME_JOINT_RANGE

EE_BODY = "link_6"  # gripper base merges into link_6 (UrdfFileCfg merge_fixed_joints)

# Multi-angle EE command ranges (radians) — Phase 1 grasp curriculum.
EE_ROLL_RANGE = (-math.pi / 2, math.pi / 2)
EE_PITCH_RANGE = (math.pi / 4, math.pi)
EE_YAW_RANGE = (-math.pi, math.pi)

# SeattleLabTable tabletop ~1.524 m along local Z (maps to world Y via table rot).
TABLE_LOCAL_Z_EXTENT_M = 1.524
TABLE_Y_EXTENSION_M = 0.6


@configclass
class E0509ReachEnvCfg(ReachEnvCfg):
    """Reach env aligned with pick_place workspace (table ~z=0.5 m)."""

    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = E0509_GRIPPER_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.num_envs = 64
        self.scene.env_spacing = 2.0

        # Table near real pick workspace (x forward from base)
        self.scene.table.init_state.pos = (0.45, 0.0, 0.0)
        # Extend tabletop 0.6 m toward world -Y (anchor +Y edge, scale local Z).
        sx, sy, sz = self.scene.table.spawn.scale or (1.0, 1.0, 1.0)
        sz *= (TABLE_LOCAL_Z_EXTENT_M + TABLE_Y_EXTENSION_M) / TABLE_LOCAL_Z_EXTENT_M
        self.scene.table.spawn.scale = (sx, sy, sz)
        tx, ty, tz = self.scene.table.init_state.pos
        self.scene.table.init_state.pos = (tx, ty - TABLE_Y_EXTENSION_M / 2.0, tz)

        for reward_name in (
            "end_effector_position_tracking",
            "end_effector_position_tracking_fine_grained",
            "end_effector_orientation_tracking",
        ):
            getattr(self.rewards, reward_name).params["asset_cfg"].body_names = [EE_BODY]

        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["joint_[1-6]"],
            scale=0.35,
            use_default_offset=True,
        )
        self.actions.gripper_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=list(GRIPPER_JOINTS),
            scale=0.5,
            use_default_offset=True,
        )

        self.commands.ee_pose.body_name = EE_BODY
        self.commands.ee_pose.ranges.pos_x = (0.25, 0.55)
        self.commands.ee_pose.ranges.pos_y = (-0.25, 0.25)
        self.commands.ee_pose.ranges.pos_z = (0.40, 0.55)
        self.commands.ee_pose.ranges.roll = EE_ROLL_RANGE
        self.commands.ee_pose.ranges.pitch = EE_PITCH_RANGE
        self.commands.ee_pose.ranges.yaw = EE_YAW_RANGE

        self.events.reset_robot_joints.params["position_range"] = (0.85, 1.15)


@configclass
class E0509ReachEnvCfg_PLAY(E0509ReachEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        # Exact pick_place home — no joint randomization in play / scene view.
        self.events.reset_robot_joints.params["position_range"] = EXACT_HOME_JOINT_RANGE
