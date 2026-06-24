"""E0509 cube lift task (pick warmup with rigid object on table)."""

import math

from isaaclab.assets import RigidObjectCfg
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from isaaclab_tasks.manager_based.manipulation.lift import mdp
from isaaclab_tasks.manager_based.manipulation.lift.lift_env_cfg import LiftEnvCfg

from isaac_e0509_pick_place.robots import E0509_GRIPPER_CFG
from isaac_e0509_pick_place.robots.home_pose import GRIPPER_JOINTS

from . import mdp_e0509

EE_BODY = "link_6"
GRIPPER_OPEN_RAD = 1.0
GRIPPER_OPEN = {name: GRIPPER_OPEN_RAD for name in GRIPPER_JOINTS}
GRIPPER_CLOSE = {name: 0.0 for name in GRIPPER_JOINTS}

# Shared with reach env (SeattleLabTable layout).
TABLE_LOCAL_Z_EXTENT_M = 1.524
TABLE_Y_EXTENSION_M = 0.6
# Raise table so its top sits at z~0.5 (base frame), matching the real robot
# workspace where objects rest ~0.5 m above the floor-mounted base. With the
# stock SeattleLabTable the top coincides with init_state.pos z (measured: cube
# rested at root z=0.02 = top 0 + half 0.02 when pos z was 0).
TABLE_TOP_Z_M = 0.40
TABLE_POS = (0.45, 0.0, TABLE_TOP_Z_M)
# Cube center on tabletop (table top 0.40 + cube half 0.02). Lowered from 0.5 because
# a top-down/tilted grasp at 0.52 needs link_6 at z~0.64, outside this arm's reach.
OBJECT_INIT_POS = (0.30, 0.0, 0.42)
LIFT_MIN_HEIGHT_M = 0.47  # cube starts ~0.42; "lifted" = raised ~0.05 (was 0.56 for cube 0.52)


def _configure_e0509_table(scene) -> None:
    """Align table with E0509 reach / pick_place workspace."""
    scene.table.init_state.pos = TABLE_POS
    sx, sy, sz = scene.table.spawn.scale or (1.0, 1.0, 1.0)
    sz *= (TABLE_LOCAL_Z_EXTENT_M + TABLE_Y_EXTENSION_M) / TABLE_LOCAL_Z_EXTENT_M
    scene.table.spawn.scale = (sx, sy, sz)
    tx, ty, tz = scene.table.init_state.pos
    scene.table.init_state.pos = (tx, ty - TABLE_Y_EXTENSION_M / 2.0, tz)


@configclass
class E0509CubeLiftEnvCfg(LiftEnvCfg):
    """Lift a dex cube with E0509 + RH-P12 (Franka Lift template)."""

    def __post_init__(self):
        super().__post_init__()

        self.scene.num_envs = 64
        self.scene.env_spacing = 2.0
        self.scene.robot = E0509_GRIPPER_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        _configure_e0509_table(self.scene)

        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["joint_[1-6]"],
            scale=0.35,
            use_default_offset=True,
        )
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=list(GRIPPER_JOINTS),
            open_command_expr=GRIPPER_OPEN,
            close_command_expr=GRIPPER_CLOSE,
        )

        self.commands.object_pose.body_name = EE_BODY
        self.commands.object_pose.ranges.pos_x = (0.30, 0.55)
        self.commands.object_pose.ranges.pos_y = (-0.25, 0.25)
        self.commands.object_pose.ranges.pos_z = (0.45, 0.65)

        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=list(OBJECT_INIT_POS), rot=[1.0, 0.0, 0.0, 0.0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )

        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/world",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path=f"{{ENV_REGEX_NS}}/Robot/{EE_BODY}",
                    name="end_effector",
                ),
            ],
        )

        self.rewards.lifting_object.params["minimal_height"] = LIFT_MIN_HEIGHT_M
        self.rewards.object_goal_tracking.params["minimal_height"] = LIFT_MIN_HEIGHT_M
        self.rewards.object_goal_tracking_fine_grained.params["minimal_height"] = LIFT_MIN_HEIGHT_M

        # Object spawn randomization (matches the model_1200 ~11% run). Narrowing
        # this to the reachable band did NOT help (training fell into reach-only),
        # so the wider spread is kept.
        self.events.reset_object_position.params["pose_range"]["x"] = (-0.08, 0.08)
        self.events.reset_object_position.params["pose_range"]["y"] = (-0.20, 0.20)
        self.events.reset_object_position.params["pose_range"]["yaw"] = (-math.pi, math.pi)
        # Keep actuator position targets aligned with home pose on every reset.
        self.events.reset_all.params["reset_joint_targets"] = True

        self.rewards.reaching_object.weight = 5.0
        # Dense grasp shaping in two stages: (1) bring both fingertips to the cube,
        # (2) reward CLOSING the gripper around it. Proximity alone let the policy
        # hover open fingers for free reward without grasping; the closed-on-object
        # term only pays out when the gripper actually shuts on the cube, which
        # bootstraps the grasp before the sparse lift reward can fire.
        self.rewards.grasp_reach = RewTerm(
            func=mdp_e0509.fingertips_object_distance,
            weight=3.0,
            params={"std": 0.06},
        )
        self.rewards.grasp_close = RewTerm(
            func=mdp_e0509.grasp_closed_on_object,
            weight=10.0,
            params={"std": 0.06},
        )
        # Softer action/velocity penalties (-1e-2) learn lifting better than the
        # Franka default -1e-1 (which suppresses the grasp motion). NOTE: training
        # still collapses deterministically at ~iter 1250 (physics blow-up once the
        # policy gets aggressive), so PPO max_iterations is capped at 1100 to stop
        # before the collapse — model_1100 is the usable checkpoint.
        self.curriculum.action_rate.params["weight"] = -1e-2
        self.curriculum.action_rate.params["num_steps"] = 30000
        self.curriculum.joint_vel.params["weight"] = -1e-2
        self.curriculum.joint_vel.params["num_steps"] = 30000

        # Match reach sim rate for consistency on the same robot asset.
        self.sim.dt = 1.0 / 60.0
        self.decimation = 2
        self.sim.render_interval = self.decimation
        self.episode_length_s = 8.0


@configclass
class E0509CubeLiftEnvCfg_PLAY(E0509CubeLiftEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 1
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        # Play: fixed cube, exact home — no train-time randomization.
        self.events.reset_object_position.params["pose_range"] = {
            "x": (0.0, 0.0),
            "y": (0.0, 0.0),
            "z": (0.0, 0.0),
        }
        self.commands.object_pose.debug_vis = True
        # Always restore exact home joints on reset (pick_place home pose).
        self.events.reset_all.params["reset_joint_targets"] = True
