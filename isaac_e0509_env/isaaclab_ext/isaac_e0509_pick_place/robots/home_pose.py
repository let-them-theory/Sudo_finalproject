"""E0509 home pose shared with ROS pick_place (degrees)."""

from __future__ import annotations

import math

# pick_place_params.yaml home is [0,0,90,0,90,0]; in Isaac that points the gripper
# sideways (+Y) because of the URDF tool-frame import. joint_6=90 rolls the tool so
# the fingers face down/forward toward the table (measured finger_dir ~ (.38,.20,-.90)),
# i.e. a grasp-ready pose matching the real robot's intent.
HOME_JOINTS_DEG: tuple[float, ...] = (0.0, 0.0, 90.0, 0.0, 90.0, 90.0)
HOME_GRIPPER_RAD: float = 0.0
# All four finger joints (r2/l1/l2 mimic r1 in URDF); we drive them explicitly
# because Isaac's URDF import does not keep the mimic coupling.
GRIPPER_JOINTS: tuple[str, ...] = (
    "gripper_rh_r1",
    "gripper_rh_r2",
    "gripper_rh_l1",
    "gripper_rh_l2",
)


def home_joint_pos_rad() -> dict[str, float]:
    """Arm + gripper initial joint positions in radians (URDF joint names)."""
    joint_pos = {f"joint_{i}": math.radians(deg) for i, deg in enumerate(HOME_JOINTS_DEG, start=1)}
    for name in GRIPPER_JOINTS:
        joint_pos[name] = HOME_GRIPPER_RAD
    return joint_pos
