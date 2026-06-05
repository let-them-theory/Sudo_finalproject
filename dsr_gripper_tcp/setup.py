import glob
import os

from setuptools import find_packages, setup

package_name = 'dsr_gripper_tcp'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob.glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dakae',
    maintainer_email='dakae2002@naver.com',
    description='Doosan RH-P12-RN(A) gripper TCP bridge, CLI example, and web dashboard (Flask/SocketIO + optional ROS2 node).',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_gripper_tcp = dsr_gripper_tcp.example_gripper_tcp:main',
            'gripper_service_node = dsr_gripper_tcp.gripper_service_node:main',
            'web_dashboard = dsr_gripper_tcp.web_dashboard:main',
            'web_dashboard_node = dsr_gripper_tcp.web_dashboard_node:main',
        ],
    },
)
