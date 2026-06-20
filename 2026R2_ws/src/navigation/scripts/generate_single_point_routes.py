#!/usr/bin/env python3
"""Generate Point 1 standalone and Point 2..5 suffix missions.

All generated missions reuse the full-FSM waypoints, profiles, actuators,
templates, pickup timing, IR retry logic, micro-sweep settings and torque
thresholds. Point 1 remains a standalone hardware test. Point 2..5 missions
skip earlier rack points and then preserve the original full-FSM flow through
Point 5.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
ROUTES_DIR = PACKAGE_DIR / "routes"
COLORS = ("blue", "red")
POINTS = range(1, 6)


def _stage_by_id(stages: list[dict[str, Any]], stage_id: str) -> dict[str, Any]:
    """Return a deep copy of one required full-FSM stage."""
    for stage in stages:
        if stage.get("id") == stage_id:
            return copy.deepcopy(stage)
    raise KeyError(f"required stage not found: {stage_id}")


def _stage_index(stages: list[dict[str, Any]], stage_id: str) -> int:
    """Return the index of one required full-FSM stage."""
    for index, stage in enumerate(stages):
        if stage.get("id") == stage_id:
            return index
    raise KeyError(f"required stage not found: {stage_id}")


def _retarget_pickup(pickup: dict[str, Any], point: int) -> None:
    """Route both search miss and post-grab IR miss to standalone cleanup."""
    pickup["on_miss"] = "advance"
    miss_target = f"slot{point}_single_miss_retreat"
    success_target = f"slot{point}_single_success_finish"

    for step in pickup.get("pickup_sequence", []):
        if step.get("id") == f"slot{point}_exit_miss":
            step["on_false"] = miss_target
        elif step.get("id") == f"slot{point}_ir_confirmed":
            step["on_true"] = success_target


def _safe_arm(color: str) -> dict[str, str]:
    """Return the rack-facing open/low state used before a pickup attempt."""
    return {
        "arm_yaw_motor": "right" if color == "blue" else "left",
        "arm_roll_motor": "up",
        "arm_gripper": "open",
        "arm_lift": "low",
        "arm_stopper": "low",
    }


def _final_arm() -> dict[str, str]:
    """Return the deterministic standalone mission finish pose."""
    return {
        "arm_yaw_motor": "front",
        "arm_roll_motor": "up",
        "arm_gripper": "open",
        "arm_lift": "low",
        "arm_stopper": "low",
    }


def build_single_point(full: dict[str, Any], color: str, point: int) -> dict[str, Any]:
    """Build one standalone mission while preserving full-FSM parameters."""
    stages = full["stages"]
    prefix = f"slot{point}"
    side_stage_id = f"{prefix}_arm_{'right' if color == 'blue' else 'left'}"

    selected_waypoints = (
        "wp_origin",
        "wp_middle",
        "wp_docking_middle",
        "wp_docking",
        f"wp_point_{point}_offset",
        f"wp_point_{point}",
    )

    pickup = _stage_by_id(stages, f"{prefix}_pickup")
    _retarget_pickup(pickup, point)

    to_middle = _stage_by_id(stages, "slot1_to_middle")
    to_middle["id"] = f"{prefix}_single_to_middle"

    to_offset = {
        "id": f"{prefix}_single_to_offset",
        "type": "action",
        "chassis": {
            "to": f"wp_point_{point}_offset",
            "profile": "red_area",
        },
    }

    finish_wait = {
        "id": f"{prefix}_single_finish_wait",
        "type": "wait",
        "duration_s": 1.0,
        "arm": _final_arm(),
    }

    standalone_stages = [
        _stage_by_id(stages, "init_wait"),
        _stage_by_id(stages, "arm_start_pose"),
        to_middle,
        _stage_by_id(stages, side_stage_id),
        to_offset,
        _stage_by_id(stages, f"{prefix}_wait_pre"),
        _stage_by_id(stages, f"{prefix}_to_point"),
        _stage_by_id(stages, f"{prefix}_settle"),
        pickup,
        # Natural search miss advances here; verify_ir miss jumps here.
        {
            "id": f"{prefix}_single_miss_retreat",
            "type": "action",
            "arm": _safe_arm(color),
            "chassis": {
                "to": f"wp_point_{point}_offset",
                "profile": "head_rack_speed",
            },
        },
        {
            "id": f"{prefix}_single_miss_finish",
            "type": "action",
            "arm": _final_arm(),
        },
        {
            "id": f"{prefix}_single_miss_wait",
            "type": "wait",
            "duration_s": 1.0,
            "arm": _final_arm(),
        },
        {"id": f"{prefix}_single_miss_done", "type": "terminate"},
        # Successful docking/release jumps over the miss cleanup to here.
        {
            "id": f"{prefix}_single_success_finish",
            "type": "action",
            "arm": _final_arm(),
        },
        finish_wait,
        {"id": f"{prefix}_single_success_done", "type": "terminate"},
    ]

    return {
        "version": full["version"],
        "frame_id": full["frame_id"],
        "angle_unit": full.get("angle_unit", "deg"),
        "waypoints": {
            name: copy.deepcopy(full["waypoints"][name]) for name in selected_waypoints
        },
        "profiles": copy.deepcopy(full["profiles"]),
        "actuators": copy.deepcopy(full["actuators"]),
        "templates": {
            name: copy.deepcopy(full["templates"][name])
            for name in ("retry_grip", "slot_pickup", "docking_release")
        },
        "stages": standalone_stages,
    }


def build_suffix_mission(full: dict[str, Any], color: str, point: int) -> dict[str, Any]:
    """Build a mission that skips Point 1..point-1 and runs through Point 5."""
    if point < 2 or point > 5:
        raise ValueError(f"suffix mission point must be 2..5, got {point}")

    stages = full["stages"]
    side_stage_id = f"slot{point}_arm_{'right' if color == 'blue' else 'left'}"
    suffix_start = _stage_index(stages, f"slot{point}_to_offset")

    # Keep the normal competition initialization and middle approach, then
    # enter the selected slot and preserve every original stage/jump to Slot 5.
    suffix_stages = [
        _stage_by_id(stages, "init_wait"),
        _stage_by_id(stages, "arm_start_pose"),
        {
            **_stage_by_id(stages, "slot1_to_middle"),
            "id": f"slot{point}_suffix_to_middle",
        },
        _stage_by_id(stages, side_stage_id),
        *copy.deepcopy(stages[suffix_start:]),
    ]

    return {
        "version": full["version"],
        "frame_id": full["frame_id"],
        "angle_unit": full.get("angle_unit", "deg"),
        "waypoints": copy.deepcopy(full["waypoints"]),
        "profiles": copy.deepcopy(full["profiles"]),
        "actuators": copy.deepcopy(full["actuators"]),
        "templates": copy.deepcopy(full["templates"]),
        "stages": suffix_stages,
    }


def main() -> None:
    """Regenerate all ten standalone route files deterministically."""
    for color in COLORS:
        source = ROUTES_DIR / color / "full_fsm.yaml"
        with source.open("r", encoding="utf-8") as stream:
            full = yaml.safe_load(stream)

        for point in POINTS:
            output = ROUTES_DIR / color / f"single_point_{point}.yaml"
            if point == 1:
                mission = build_single_point(full, color, point)
                description = (
                    f"Standalone {color} Point 1: origin -> middle -> offset "
                    "-> pickup -> docking -> terminate."
                )
            else:
                mission = build_suffix_mission(full, color, point)
                description = (
                    f"{color} Point {point} suffix: skip Point 1..{point - 1}, "
                    f"then run Point {point}..5."
                )
            header = (
                f"# Generated from routes/{color}/full_fsm.yaml by "
                "scripts/generate_single_point_routes.py.\n"
                f"# {description}\n"
            )
            with output.open("w", encoding="utf-8") as stream:
                stream.write(header)
                yaml.safe_dump(mission, stream, sort_keys=False, allow_unicode=True)
            print(output.relative_to(PACKAGE_DIR))


if __name__ == "__main__":
    main()
