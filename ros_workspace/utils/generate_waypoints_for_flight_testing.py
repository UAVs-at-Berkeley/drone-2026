"""
Generate 7 test waypoints around a central GPS coordinate.

Places 7 points evenly on a circle, then perturbs each by a random offset.
Outputs ``environment.waypoints.points`` entries as ``[lat, long, alt_m]`` lines for mission YAML.

Usage:
    python3 generate_waypoints_for_flight_testing.py <center_lat> <center_lon> <altitude>
"""

import math
import random
import sys

NUM_WAYPOINTS = 7
RADIUS_METERS = 100.0
PERTURBATION_MAX = 5.0  # meters, uniform(0, PERTURBATION_MAX)

# Approximate meters-per-degree at mid-latitudes
METERS_PER_DEG_LAT = 111320.0


def meters_per_deg_lon(lat):
    return 111320.0 * math.cos(math.radians(lat))


def generate_waypoints(center_lat, center_lon, altitude):
    waypoints = []
    for i in range(NUM_WAYPOINTS):
        angle = 2 * math.pi * i / NUM_WAYPOINTS

        # Point on circle
        dx = RADIUS_METERS * math.cos(angle)
        dy = RADIUS_METERS * math.sin(angle)

        # Random perturbation
        dx += random.uniform(0, PERTURBATION_MAX) * random.choice([-1, 1])
        dy += random.uniform(0, PERTURBATION_MAX) * random.choice([-1, 1])
        dz = random.uniform(0, PERTURBATION_MAX) * random.choice([-1, 1])

        lat = center_lat + dy / METERS_PER_DEG_LAT
        lon = center_lon + dx / meters_per_deg_lon(center_lat)
        alt = altitude + dz

        waypoints.append((lat, lon, alt))

    return waypoints


def main():
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <center_lat> <center_lon> <altitude>")
        sys.exit(1)

    center_lat = float(sys.argv[1])
    center_lon = float(sys.argv[2])
    altitude = float(sys.argv[3])

    waypoints = generate_waypoints(center_lat, center_lon, altitude)

    lats = [f"{wp[0]:.7f}" for wp in waypoints]
    lons = [f"{wp[1]:.7f}" for wp in waypoints]
    alts = [f"{wp[2]:.1f}" for wp in waypoints]

    print("Paste under environment.waypoints.points in your mission YAML:\n")
    for i in range(len(waypoints)):
        print(f"      - [{lats[i]}, {lons[i]}, {alts[i]}]")


if __name__ == "__main__":
    main()
