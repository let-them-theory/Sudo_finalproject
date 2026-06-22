"""E0509 cube lift task (pick warmup with rigid object on table)."""

from isaaclab.assets import RigidObjectCfg
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

EE_BODY = "link_6"
GRIPPER_OPEN_RAD = 1.0
GRIPPER_OPEN = {name: GRIPPER_OPEN_RAD for name in GRIPPER_JOINTS}
GRIPPER_CLOSE = {name: 0.0 for name in GRIPPER_JOINTS}

# Shared with reach env (SeattleLabTable layout).
TABLE_LOCAL_Z_EXTENT_M = 1.524
TABLE_Y_EXTENSION_M = 0.6
TABLE_POS = (0.45, 0.0, 0.0)
# Cube center on tabletop (reach EE z ~ 0.40-0.55).
OBJECT_INIT_POS = (0.45, 0.0, 0.52)
LIFT_MIN_HEIGHT_M = 0.56


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

        self.events.reset_object_position.params["pose_range"]["x"] = (-0.08, 0.08)
        self.events.reset_object_position.params["pose_range"]["y"] = (-0.20, 0.20)
        # Keep actuator position targets aligned with home pose on every reset.
        self.events.reset_all.params["reset_joint_targets"] = True

        # Encourage approach before lift/grasp (default weight=1.0 is too weak for E0509).
        self.rewards.reaching_object.weight = 5.0
        # Softer action penalties so reach-pretrained arm motion is not suppressed early.
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
