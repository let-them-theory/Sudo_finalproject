#!/usr/bin/env python3
"""Open Isaac Lab GUI (lighter than ./isaaclab.sh -s full Isaac Sim)."""

from isaaclab.app import AppLauncher

import argparse

parser = argparse.ArgumentParser(description="Isaac Lab GUI")
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(rendering_mode="performance")
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

print("[OK] Isaac Lab GUI running. Close the window to exit.", flush=True)
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
print("[DONE]", flush=True)
