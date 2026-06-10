#!/usr/bin/env python3
"""
ROS2 Bridge for Digital Twin (별도 프로세스로 실행)

ROS2에서 joint_states를 받아서 공유 파일에 저장합니다.
Isaac Sim digital_twin.py가 이 파일을 읽어서 로봇을 동기화합니다.

특징:
- Arm (joint_1~6)과 Gripper (gripper_rh_*) 조인트를 누적 저장
- 두 개의 publisher가 같은 토픽에 발행해도 모두 합쳐서 저장

사용법:
    # 터미널 2: ROS2 Bridge 실행 (ROS2 환경에서)
    source /opt/ros/humble/setup.bash
    source ~/doosan_ws/install/setup.bash
    python3 digital_twin_bridge.py
"""

import json
import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# 공유 파일 경로
SHARED_FILE = '/tmp/doosan_joint_states.json'


class JointStateBridge(Node):
    """Joint states를 파일로 저장하는 브릿지 노드

    Arm과 Gripper가 각각 joint_states를 발행하므로,
    조인트 이름 기반으로 누적 저장합니다.
    """

    def __init__(self, namespace='dsr01'):
        super().__init__('joint_state_bridge')
        self.namespace = namespace

        # 조인트 위치를 누적 저장하는 딕셔너리
        self.joint_positions = {}
        self.joint_velocities = {}

        # Subscriber
        topic_name = f'/{namespace}/joint_states'
        self.subscription = self.create_subscription(
            JointState,
            topic_name,
            self.joint_state_callback,
            10
        )

        self.get_logger().info(f'Joint State Bridge started')
        self.get_logger().info(f'Subscribed to: {topic_name}')
        self.get_logger().info(f'Writing to: {SHARED_FILE}')
        self.get_logger().info(f'Accumulating arm + gripper joints')

        self.count = 0

    def joint_state_callback(self, msg):
        """Joint state를 누적하여 파일에 저장"""
        # 새로운 조인트 데이터를 누적
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if msg.velocity and i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

        # 누적된 전체 데이터를 저장
        names = list(self.joint_positions.keys())
        positions = [self.joint_positions[n] for n in names]
        velocities = [self.joint_velocities.get(n, 0.0) for n in names]

        data = {
            'timestamp': time.time(),
            'names': names,
            'positions': positions,
            'velocities': velocities,
        }

        try:
            with open(SHARED_FILE, 'w') as f:
                json.dump(data, f)

            self.count += 1
            if self.count % 100 == 0:
                self.get_logger().info(f'Published {self.count} joint states ({len(names)} joints)')

        except Exception as e:
            self.get_logger().error(f'Failed to write: {e}')


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', '-n', default='dsr01')
    args = parser.parse_args()

    rclpy.init()
    node = JointStateBridge(namespace=args.namespace)

    print('=' * 60)
    print('  ROS2 Joint State Bridge')
    print('=' * 60)
    print(f'  Namespace: {args.namespace}')
    print(f'  Output: {SHARED_FILE}')
    print('=' * 60)
    print('  Press Ctrl+C to exit')
    print('=' * 60)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # 파일 정리
        if os.path.exists(SHARED_FILE):
            os.remove(SHARED_FILE)


if __name__ == '__main__':
    main()
