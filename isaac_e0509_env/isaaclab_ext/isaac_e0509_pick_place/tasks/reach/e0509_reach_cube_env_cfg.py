"""E0509 reach toward dex cube on table (approach task that actually works)."""

import math

from isaaclab.assets import RigidObjectCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from isaaclab_tasks.manager_based.manipulation.reach.reach_env_cfg import ReachSceneCfg

from .e0509_reach_env_cfg import E0509ReachEnvCfg

# Same layout as lift task.
OBJECT_INIT_POS = (0.45, 0.0, 0.52)


@configclass
class E0509ReachCubeSceneCfg(ReachSceneCfg):
    """Reach scene plus a visible dex cube on the table."""

    object = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Object",
        init_state=RigidObjectCfg.InitialStateCfg(pos=list(OBJECT_INIT_POS), rot=[1.0, 0.0, 0.0, 0.0]),
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
            scale=(0.8, 0.8, 0.8),
            rigid_props=RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=1,
                disable_gravity=False,
            ),
        ),
    )


@configclass
class E0509ReachCubeEnvCfg(E0509ReachEnvCfg):
    """Reach EE goals sampled around the cube — same obs/action as reach (34-dim policy)."""

    scene: E0509ReachCubeSceneCfg = E0509ReachCubeSceneCfg(num_envs=64, env_spacing=2.0)

    def __post_init__(self):
        super().__post_init__()

        # Goals centered on cube (not random table-wide like generic reach).
        self.commands.ee_pose.ranges.pos_x = (0.40, 0.50)
        self.commands.ee_pose.ranges.pos_y = (-0.10, 0.10)
        self.commands.ee_pose.ranges.pos_z = (0.48, 0.56)
        self.commands.ee_pose.ranges.pitch = (math.pi, math.pi)


@configclass
class E0509ReachCubeEnvCfg_PLAY(E0509ReachCubeEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 1
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        # Fixed goal above cube center for predictable play.
        self.commands.ee_pose.ranges.pos_x = (0.45, 0.45)
        self.commands.ee_pose.ranges.pos_y = (0.0, 0.0)
        self.commands.ee_pose.ranges.pos_z = (0.52, 0.52)
