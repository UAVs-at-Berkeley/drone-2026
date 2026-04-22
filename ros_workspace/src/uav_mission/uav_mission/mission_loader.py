"""
Load and validate mission YAML for central_command_node.

Schema: top-level key ``mission`` with ``steps`` as a list. Each step is either a string
(step id) or a mapping with required ``id`` and optional step-specific fields.

Allowed step ids are documented in ``missions/README.md`` (installed under share).
"""

from __future__ import annotations

import os
from typing import Any, Dict, List
import yaml

ALLOWED_STEP_IDS = frozenset(
    {
        "takeoff",
        "time_trial",
        "object_localization",
        "return_to_home",
        "land",
        "payload_drop",
    }
)


def normalize_steps(raw_steps: List[Any]) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for i, item in enumerate(raw_steps):
        if isinstance(item, str):
            out.append({"id": item.strip()})
        elif isinstance(item, dict):
            if "id" not in item:
                raise ValueError("Mission step %d: dict entry must have 'id' key" % i)
            sid = item["id"]
            if not isinstance(sid, str) or not sid.strip():
                raise ValueError("Mission step %d: 'id' must be a non-empty string" % i)
            row = dict(item)
            row["id"] = sid.strip()
            out.append(row)
        else:
            raise ValueError(
                "Mission step %d: expected string or mapping, got %s"
                % (i, type(item).__name__)
            )
    return out


def load_mission_file(path: str) -> List[Dict[str, Any]]:
    if not path or not path.strip():
        raise ValueError("mission file path is empty")
    path = os.path.abspath(os.path.expanduser(path.strip()))
    if not os.path.isfile(path):
        raise FileNotFoundError("Mission file not found: %s" % path)
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if data is None:
        raise ValueError("Mission file is empty: %s" % path)
    if not isinstance(data, dict):
        raise ValueError("Mission file root must be a mapping")
    mission = data.get("mission")
    if not isinstance(mission, dict):
        raise ValueError("Mission file must contain a 'mission:' mapping")
    raw_steps = mission.get("steps")
    if not isinstance(raw_steps, list) or len(raw_steps) == 0:
        raise ValueError("mission.steps must be a non-empty list")
    steps = normalize_steps(raw_steps)
    for i, s in enumerate(steps):
        sid = s["id"]
        if sid not in ALLOWED_STEP_IDS:
            raise ValueError(
                "Unknown mission step id %r at index %d. Allowed: %s"
                % (sid, i, ", ".join(sorted(ALLOWED_STEP_IDS)))
            )
    return steps
