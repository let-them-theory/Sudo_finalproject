"""E0509 scene spawn config (URDF unchanged; stiffer drives for Isaac Sim viewing)."""

from __future__ import annotations

import isaaclab.sim as sim_utils

from .e0509 import E0509_GRIPPER_CFG

# Import-time joint drives (URDF has no arm stiffness/damping).
_JOINT_DRIVE = sim_utils.UrdfConverterCfg.JointDriveCfg(
    gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=8000.0, damping=80.0)
)

_spawn = E0509_GRIPPER_CFG.spawn.replace(joint_drive=_JOINT_DRIVE)

_actuators = {
    "arm": E0509_GRIPPER_CFG.actuators["arm"].replace(
        effort_limit_sim=194.0,
        stiffness=8000.0,
        damping=80.0,
    ),
    "gripper": E0509_GRIPPER_CFG.actuators["gripper"].replace(
        stiffness=2000.0,
        damping=100.0,
    ),
}

E0509_SCENE_CFG = E0509_GRIPPER_CFG.replace(spawn=_spawn, actuators=_actuators)
