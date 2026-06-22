"""Transfer RSL-RL policy weights from E0509 Reach to E0509 Lift."""

from __future__ import annotations

import torch

REACH_POLICY_OBS_DIM = 34
LIFT_POLICY_OBS_DIM = 37
JOINT_OBS_DIM = 20  # joint_pos_rel + joint_vel_rel
OBJECT_POS_DIM = 3
POSE_COMMAND_DIM = 7
ACTION_OBS_DIM = 7


def _map_reach_input_to_lift(reach_weight: torch.Tensor, lift_weight: torch.Tensor) -> torch.Tensor:
    """Map reach actor/critic input weights onto lift observation layout."""
    mapped = lift_weight.clone()
    mapped[:, :JOINT_OBS_DIM] = reach_weight[:, :JOINT_OBS_DIM]

    target_start = JOINT_OBS_DIM + OBJECT_POS_DIM
    reach_pose_start = JOINT_OBS_DIM
    mapped[:, target_start : target_start + POSE_COMMAND_DIM] = reach_weight[
        :, reach_pose_start : reach_pose_start + POSE_COMMAND_DIM
    ]

    mapped[:, LIFT_POLICY_OBS_DIM - ACTION_OBS_DIM :] = reach_weight[:, REACH_POLICY_OBS_DIM - ACTION_OBS_DIM :]
    return mapped


def transfer_reach_to_lift_state_dict(
    lift_state_dict: dict[str, torch.Tensor],
    reach_checkpoint: str,
    device: str = "cpu",
) -> dict[str, torch.Tensor]:
    """Build a lift state dict initialized from a reach checkpoint.

    Reach and lift must share the same MLP hidden sizes (``[128, 128]``). Only the
    first linear layer differs in input width (34 vs 37). Shared columns are copied;
    object-position inputs keep lift initialization.
    """
    reach_state = torch.load(reach_checkpoint, map_location=device, weights_only=False)["model_state_dict"]
    merged = {key: value.clone() for key, value in lift_state_dict.items()}

    for prefix in ("actor", "critic"):
        in_key = f"{prefix}.0.weight"
        bias_key = f"{prefix}.0.bias"
        merged[in_key] = _map_reach_input_to_lift(reach_state[in_key], lift_state_dict[in_key])
        merged[bias_key] = reach_state[bias_key].clone()

        for layer_idx in (2, 4):
            for suffix in ("weight", "bias"):
                key = f"{prefix}.{layer_idx}.{suffix}"
                if key in reach_state and reach_state[key].shape == lift_state_dict[key].shape:
                    merged[key] = reach_state[key].clone()

    return merged


def load_reach_into_lift_runner(runner, reach_checkpoint: str, device: str = "cpu") -> None:
    """Load reach arm policy weights into a lift OnPolicyRunner (fresh optimizer)."""
    policy = runner.alg.policy
    merged = transfer_reach_to_lift_state_dict(policy.state_dict(), reach_checkpoint, device=device)
    resumed = policy.load_state_dict(merged, strict=False)
    print(f"[INFO] Initialized lift policy from reach checkpoint: {reach_checkpoint} (resumed={resumed})")
