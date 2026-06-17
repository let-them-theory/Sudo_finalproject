#!/usr/bin/env python3
"""Set E0509 home pose in Isaac Sim after URDF import.

Usage (Isaac Sim Script Editor):
  1. Import e0509_gripper_isaac.urdf
  2. Press Play once
  3. Window -> Script Editor -> open this file -> Run

Or from terminal (isaacsim already running is not required):
  isaacsim --exec ~/sudo_ws/src/isaac_e0509_env/scripts/apply_e0509_home_isaac.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import omni.usd
from isaacsim.core.prims import Articulation
from pxr import UsdPhysics

# pick_place_params.yaml home_joints (deg)
HOME_JOINTS_DEG = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)
HOME_GRIPPER_RAD = 0.0


def _home_targets(dof_names: list[str]) -> np.ndarray:
    home_map = {f"joint_{i}": math.radians(deg) for i, deg in enumerate(HOME_JOINTS_DEG, start=1)}
    home_map["gripper_rh_r1"] = HOME_GRIPPER_RAD
    return np.array([home_map.get(name, 0.0) for name in dof_names], dtype=np.float32)


def _articulation_roots() -> list[str]:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return []
    roots: list[str] = []
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            roots.append(str(prim.GetPath()))
    return roots


def apply_home(prim_path: str | None = None) -> None:
    paths = [prim_path] if prim_path else _articulation_roots()
    if not paths:
        raise RuntimeError("No articulation found. Import e0509_gripper_isaac.urdf first.")

    for path in paths:
        art = Articulation(prim_paths_expr=path)
        art.initialize()
        targets = _home_targets(list(art.dof_names))
        art.set_joint_positions(targets)
        art.set_joint_position_targets(targets)
        print(f"[OK] Home applied: {path}")
        print(f"     dof_names={list(art.dof_names)}")
        print(f"     targets(rad)={targets.tolist()}")


if __name__ == "__main__":
  # Optional: pass articulation prim path as argv[1]
    apply_home(sys.argv[1] if len(sys.argv) > 1 else None)
