#!/usr/bin/env python3
"""Helpers for parsing mission YAML at launch time."""

import json
from typing import Any, Dict, List

from launch.substitutions import LaunchConfiguration

from uav_mission.mission_loader import load_mission_data


def _split_waypoints(environment: Dict[str, Any]) -> Dict[str, List[float]]:
    points = environment.get("waypoints", {}).get("points", [])
    lats = [float(p[0]) for p in points]
    lons = [float(p[1]) for p in points]
    alts = [float(p[2]) for p in points]
    return {"lats": lats, "lons": lons, "alts": alts}


def _target(environment: Dict[str, Any], key: str) -> Dict[str, float]:
    vals = environment.get(key, [0.0, 0.0, 0.0])
    return {
        "lat": float(vals[0]),
        "lon": float(vals[1]),
        "alt": float(vals[2]),
    }


def mission_parameter_bundle(context) -> Dict[str, Dict[str, Any]]:
    mission_file = LaunchConfiguration("mission_file").perform(context).strip()
    if mission_file:
        mission = load_mission_data(mission_file)
        environment = mission["environment"]
        steps = mission["steps"]
    else:
        environment = {
            "Geofence": {"points": []},
            "waypoints": {"points": []},
            "red_target": [0.0, 0.0, 0.0],
            "x_target": [0.0, 0.0, 0.0],
            "number_target": [0.0, 0.0, 0.0],
        }
        steps = [{"id": "takeoff"}]

    wp = _split_waypoints(environment)
    red = _target(environment, "red_target")
    x_target = _target(environment, "x_target")
    number = _target(environment, "number_target")
    geofence_points = environment.get("Geofence", {}).get("points", [])

    return {
        "central_command": {
            "mission_steps_json": json.dumps(steps),
        },
        "waypoint": {
            "waypoint_lats": wp["lats"],
            "waypoint_lons": wp["lons"],
            "waypoint_alts": wp["alts"],
        },
        "time_trial": {
            "waypoint_lats": wp["lats"],
            "waypoint_lons": wp["lons"],
            "waypoint_alts": wp["alts"],
        },
        "payload_drop": {
            "red_target_latitude_deg": red["lat"],
            "red_target_longitude_deg": red["lon"],
            "red_target_altitude_m": red["alt"],
            "x_target_latitude_deg": x_target["lat"],
            "x_target_longitude_deg": x_target["lon"],
            "x_target_altitude_m": x_target["alt"],
            "number_target_latitude_deg": number["lat"],
            "number_target_longitude_deg": number["lon"],
            "number_target_altitude_m": number["alt"],
        },
        "object_localization": {
            "red_target_latitude_deg": red["lat"],
            "red_target_longitude_deg": red["lon"],
            "red_target_altitude_m": red["alt"],
            "x_target_latitude_deg": x_target["lat"],
            "x_target_longitude_deg": x_target["lon"],
            "x_target_altitude_m": x_target["alt"],
            "number_target_latitude_deg": number["lat"],
            "number_target_longitude_deg": number["lon"],
            "number_target_altitude_m": number["alt"],
            "geofence_points_json": json.dumps(geofence_points),
        },
    }
