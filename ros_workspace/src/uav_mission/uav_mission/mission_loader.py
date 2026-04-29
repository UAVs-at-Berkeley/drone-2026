"""
Load and validate mission YAML for central_command_node.

``time_trial`` uses only ``environment.waypoints.points`` (each ``[lat, long, alt_m]``);
do not set latitudes/longitudes/altitudes on the step in YAML.

Schema: top-level key ``mission`` with ``steps`` as a list. Each step is either a string
(step id) or a mapping with required ``id`` and optional step-specific fields (except
legacy time_trial inline arrays, which are rejected).

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


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _validate_coordinate(value: Any, field_name: str) -> None:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError("%s must be a list of exactly 3 numbers: [lat, long, alt_m]" % field_name)
    for i, item in enumerate(value):
        if not _is_number(item):
            raise ValueError("%s[%d] must be numeric, got %s" % (field_name, i, type(item).__name__))


def _validate_coordinate_points_list(value: Any, field_name: str) -> None:
    if not isinstance(value, list):
        raise ValueError("%s must be a list of coordinates" % field_name)
    for i, point in enumerate(value):
        _validate_coordinate(point, "%s[%d]" % (field_name, i))


def validate_environment(raw_environment: Any) -> None:
    if not isinstance(raw_environment, dict):
        raise ValueError("Mission file must contain an 'environment:' mapping")

    geofence = raw_environment.get("Geofence")
    if not isinstance(geofence, dict):
        raise ValueError("environment.Geofence must be a mapping with a 'points' list")
    _validate_coordinate_points_list(geofence.get("points"), "environment.Geofence.points")

    waypoints = raw_environment.get("waypoints")
    if not isinstance(waypoints, dict):
        raise ValueError("environment.waypoints must be a mapping with a 'points' list")
    _validate_coordinate_points_list(waypoints.get("points"), "environment.waypoints.points")

    _validate_coordinate(raw_environment.get("red_target"), "environment.red_target")
    _validate_coordinate(raw_environment.get("x_target"), "environment.x_target")
    _validate_coordinate(raw_environment.get("number_target"), "environment.number_target")


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


def _expand_time_trial_from_env(steps: List[Dict[str, Any]], environment: Dict[str, Any]) -> None:
    """
    Time trial uses a single source of truth: ``environment.waypoints.points`` as
    ``[lat, long, alt_m]`` triples. This copies them into the step as parallel lists for
    ``StartTimeTrial`` goals (internal step fields only; do not
    set those keys in mission YAML).
    """
    for s in steps:
        if s.get("id") != "time_trial":
            continue
        for k in ("latitudes", "longitudes", "altitudes"):
            if k in s:
                raise ValueError(
                    "time_trial: do not set %r in mission YAML. "
                    "Use environment.waypoints.points only (list of [lat, long, alt_m])." % k
                )
        points = environment.get("waypoints", {}).get("points", [])
        if not isinstance(points, list) or not points:
            raise ValueError(
                "time_trial step requires a non-empty environment.waypoints.points list "
                "of [lat, long, alt_m] coordinates"
            )
        s["latitudes"] = [float(p[0]) for p in points]
        s["longitudes"] = [float(p[1]) for p in points]
        s["altitudes"] = [float(p[2]) for p in points]


def load_mission_file(path: str) -> List[Dict[str, Any]]:
    mission = load_mission_data(path)
    return mission["steps"]


def load_mission_data(path: str) -> Dict[str, Any]:
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
    environment = data.get("environment")
    validate_environment(environment)
    raw_steps = mission.get("steps")
    if not isinstance(raw_steps, list) or len(raw_steps) == 0:
        raise ValueError("mission.steps must be a non-empty list")
    steps = normalize_steps(raw_steps)
    _expand_time_trial_from_env(steps, environment)
    for i, s in enumerate(steps):
        sid = s["id"]
        if sid not in ALLOWED_STEP_IDS:
            raise ValueError(
                "Unknown mission step id %r at index %d. Allowed: %s"
                % (sid, i, ", ".join(sorted(ALLOWED_STEP_IDS)))
            )
    return {"environment": environment, "steps": steps}
