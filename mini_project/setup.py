from setuptools import find_packages, setup
import os
from glob import glob

# ROS 2 패키지 이름. 여러 곳에서 반복 사용하므로 상수로 분리한다.
package_name = 'dsr_realsense_pick_place'


def _recursive_data_files(src_dir):
    """src_dir(소스 상대경로) 이하 모든 파일을 share/패키지/<같은 구조>로 설치하는
    data_files 엔트리 목록을 만든다. web 프론트 dist(assets/ 하위포함) 설치용."""
    entries = []
    for root, _dirs, files in os.walk(src_dir):
        if not files:
            continue
        dest = os.path.join('share', package_name, root)
        entries.append((dest, [os.path.join(root, f) for f in files]))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 인덱스와 share 디렉터리에 패키지 메타데이터를 설치한다.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch, config 파일도 설치본에서 바로 찾을 수 있도록 함께 복사한다.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh') + glob('scripts/*.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
        # user web 키오스크 백엔드(스크립트)를 share에 설치 — launch에서 절대경로로 실행.
        (os.path.join('share', package_name, 'web_kiosk', 'backend'),
         glob('web_kiosk/backend/*.py')),
    ] + _recursive_data_files('web_kiosk/frontend/dist'),  # 빌드된 프론트(dist) 통째 설치
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'ultralytics',
        'PyQt5',
        'pyrealsense2',
    ],
    zip_safe=True,
    maintainer='hyunwook',
    maintainer_email='hyunwook@todo.com',
    description='Doosan E0509 pick and place with RealSense, YOLO, and GUI object selection',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run 으로 실행할 엔트리포인트 등록
            'object_detector = dsr_realsense_pick_place.object_detector:main',
            'pick_place_node = dsr_realsense_pick_place.pick_place_node:main',
            'gui_node = dsr_realsense_pick_place.gui_node:main',
            'gripper_node = dsr_realsense_pick_place.gripper_node:main',
            'ultrasonic_node = dsr_realsense_pick_place.ultrasonic_node:main',
        ],
    },
)
