#!/usr/bin/env python3
"""Copy E0509+gripper URDF and rewrite package:// mesh paths for Isaac Sim."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

_SRC_ROOT = Path(__file__).resolve().parents[2]
_ENV_ROOT = Path(__file__).resolve().parents[1]

DEFAULT_SRC = _SRC_ROOT / "e0509_gripper_description/config/curobo/e0509_gripper.urdf"
DEFAULT_DST = _ENV_ROOT / "assets/urdf/e0509_gripper_isaac.urdf"

PACKAGE_MAP = {
    "dsr_description2": _SRC_ROOT / "doosan-robot2/dsr_description2",
    "rh_p12_rn_a_description": _SRC_ROOT / "RH-P12-RN-A/rh_p12_rn_a_description",
}


def rewrite_package_uris(text: str) -> str:
    def _replace(match: re.Match[str]) -> str:
        pkg = match.group(1)
        rel = match.group(2)
        if pkg not in PACKAGE_MAP:
            raise KeyError(f"Unknown ROS package in URDF: {pkg}")
        abs_path = (PACKAGE_MAP[pkg] / rel).resolve()
        if not abs_path.exists():
            raise FileNotFoundError(f"Mesh not found: {abs_path}")
        return f'filename="{abs_path}"'

    return re.sub(r'filename="package://([^/]+)/([^"]+)"', _replace, text)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--src", type=Path, default=DEFAULT_SRC)
    parser.add_argument("--dst", type=Path, default=DEFAULT_DST)
    args = parser.parse_args()

    if not args.src.exists():
        raise FileNotFoundError(f"Source URDF not found: {args.src}")

    for pkg, root in PACKAGE_MAP.items():
        if not root.exists():
            raise FileNotFoundError(f"ROS package root missing ({pkg}): {root}")

    text = args.src.read_text(encoding="utf-8")
    out = rewrite_package_uris(text)

    args.dst.parent.mkdir(parents=True, exist_ok=True)
    args.dst.write_text(out, encoding="utf-8")
    print(f"Wrote Isaac-ready URDF: {args.dst}")


if __name__ == "__main__":
    main()
