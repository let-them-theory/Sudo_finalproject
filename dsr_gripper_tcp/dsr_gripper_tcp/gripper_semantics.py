from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class SemanticStateSnapshot:
    ready: bool
    torque_enabled: bool
    moving: bool
    in_position: bool
    status: int
    moving_status: int
    present_position: int
    goal_position: int
    present_current: int
    current_limit: int
    present_velocity: int
    present_temperature: int
    grasp_detected: bool
    object_lost: bool
    status_text: str


class GripperSemanticEvaluator:
    def __init__(
        self,
        grasp_current_threshold: int,
        object_lost_current_threshold: int,
        object_lost_position_delta: int,
    ) -> None:
        self._grasp_current_threshold = int(grasp_current_threshold)
        self._object_lost_current_threshold = int(object_lost_current_threshold)
        self._object_lost_position_delta = int(object_lost_position_delta)
        self._had_grasp = False
        self._last_grasp_position: int | None = None

    def evaluate(self, bridge_state, goal_position: int, current_limit: int, status_text: str) -> SemanticStateSnapshot:
        torque_enabled = bool(bridge_state.torque_enabled)
        moving = bool(bridge_state.moving)
        in_position = bool(bridge_state.in_position)
        status = int(bridge_state.status)
        present_position = int(bridge_state.present_position)
        present_current = int(bridge_state.present_current)
        current_abs = abs(present_current)

        if not torque_enabled or status != 0:
            self.reset_grasp_history()

        grasp_detected = False
        object_lost = False

        if torque_enabled and not moving:
            grasp_detected = (
                current_abs >= self._grasp_current_threshold
                or current_abs >= int(int(current_limit) * 0.9)
            )

        if grasp_detected:
            self._had_grasp = True
            self._last_grasp_position = present_position
        elif self._had_grasp and torque_enabled:
            if current_abs <= self._object_lost_current_threshold:
                if self._last_grasp_position is None:
                    object_lost = True
                else:
                    position_delta = abs(present_position - self._last_grasp_position)
                    object_lost = position_delta >= self._object_lost_position_delta

        return SemanticStateSnapshot(
            ready=torque_enabled,
            torque_enabled=torque_enabled,
            moving=moving,
            in_position=in_position,
            status=status,
            moving_status=int(bridge_state.moving_status),
            present_position=present_position,
            goal_position=int(goal_position),
            present_current=present_current,
            current_limit=int(current_limit),
            present_velocity=int(bridge_state.present_velocity),
            present_temperature=int(bridge_state.present_temperature),
            grasp_detected=grasp_detected,
            object_lost=object_lost,
            status_text=status_text,
        )

    def reset_grasp_history(self) -> None:
        self._had_grasp = False
        self._last_grasp_position = None
