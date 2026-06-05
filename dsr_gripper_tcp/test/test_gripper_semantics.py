import unittest
from types import SimpleNamespace

from dsr_gripper_tcp.gripper_semantics import GripperSemanticEvaluator


def make_bridge_state(
    *,
    torque_enabled=True,
    moving=0,
    moving_status=1,
    status=0,
    present_position=500,
    present_current=100,
    present_velocity=0,
    present_temperature=30,
):
    return SimpleNamespace(
        torque_enabled=torque_enabled,
        moving=moving,
        moving_status=moving_status,
        status=status,
        present_position=present_position,
        present_current=present_current,
        present_velocity=present_velocity,
        present_temperature=present_temperature,
        in_position=bool(moving_status & 0x01),
    )


class GripperSemanticEvaluatorTests(unittest.TestCase):
    def test_grasp_detection_uses_current_thresholds(self):
        evaluator = GripperSemanticEvaluator(
            grasp_current_threshold=300,
            object_lost_current_threshold=80,
            object_lost_position_delta=80,
        )

        snapshot = evaluator.evaluate(
            make_bridge_state(present_current=320),
            goal_position=700,
            current_limit=400,
            status_text='ok',
        )

        self.assertTrue(snapshot.grasp_detected)
        self.assertFalse(snapshot.object_lost)

    def test_object_lost_requires_prior_grasp_low_current_and_position_delta(self):
        evaluator = GripperSemanticEvaluator(
            grasp_current_threshold=300,
            object_lost_current_threshold=80,
            object_lost_position_delta=80,
        )

        evaluator.evaluate(
            make_bridge_state(present_position=600, present_current=350),
            goal_position=700,
            current_limit=400,
            status_text='grasped',
        )
        snapshot = evaluator.evaluate(
            make_bridge_state(present_position=480, present_current=40),
            goal_position=700,
            current_limit=400,
            status_text='released',
        )

        self.assertFalse(snapshot.grasp_detected)
        self.assertTrue(snapshot.object_lost)

    def test_torque_off_resets_grasp_history(self):
        evaluator = GripperSemanticEvaluator(
            grasp_current_threshold=300,
            object_lost_current_threshold=80,
            object_lost_position_delta=80,
        )

        evaluator.evaluate(
            make_bridge_state(present_position=610, present_current=330),
            goal_position=700,
            current_limit=400,
            status_text='grasped',
        )
        torque_off = evaluator.evaluate(
            make_bridge_state(torque_enabled=False, present_position=520, present_current=0),
            goal_position=700,
            current_limit=400,
            status_text='torque off',
        )
        after_reset = evaluator.evaluate(
            make_bridge_state(present_position=430, present_current=20),
            goal_position=700,
            current_limit=400,
            status_text='idle',
        )

        self.assertFalse(torque_off.grasp_detected)
        self.assertFalse(torque_off.object_lost)
        self.assertFalse(after_reset.object_lost)


if __name__ == '__main__':
    unittest.main()
