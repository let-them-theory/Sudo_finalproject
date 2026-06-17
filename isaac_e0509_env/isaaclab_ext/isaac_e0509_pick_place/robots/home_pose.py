"""E0509 home pose shared with ROS pick_place (degrees)."""

from __future__ import annotations

import math

# pick_place_params.yaml → home_joints
HOME_JOINTS_DEG: tuple[float, ...] = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)
HOME_GRIPPER_RAD: float = 0.0


def home_joint_pos_rad() -> dict[str, float]:
    """Arm + gripper initial joint positions in radians (URDF joint names)."""
    joint_pos = {f"joint_{i}": math.radians(deg) for i, deg in enumerate(HOME_JOINTS_DEG, start=1)}
    joint_pos["gripper_rh_r1"] = HOME_GRIPPER_RAD
    return joint_pos
