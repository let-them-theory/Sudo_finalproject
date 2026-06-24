"""Installation script for the isaac_e0509_pick_place Isaac Lab extension."""

from setuptools import find_packages, setup

setup(
    name="isaac_e0509_pick_place",
    packages=find_packages(),
    author="sudo_ws",
    maintainer="sudo_ws",
    url="https://github.com/isaac-sim/IsaacLab",
    version="0.1.0",
    description="Doosan E0509 + RH-P12 gripper environments for Isaac Lab",
    keywords=["doosan", "e0509", "manipulation", "reach", "isaaclab"],
    install_requires=[],
    license="Apache-2.0",
    include_package_data=True,
    python_requires=">=3.10",
    classifiers=[
        "Natural Language :: English",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Isaac Sim :: 5.0.0",
        "Isaac Sim :: 5.1.0",
    ],
    zip_safe=False,
)
