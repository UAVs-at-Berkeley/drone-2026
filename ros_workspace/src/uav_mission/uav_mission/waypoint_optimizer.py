#!/usr/bin/env python3
"""
Waypoint Order Optimizer
========================
Given a set of waypoints, find the ordering that minimizes total travel distance.

With 7 waypoints there are 7! = 5040 possible orderings.
We try every single one and pick the shortest.
"""

import math
from itertools import permutations


def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate distance in meters between two GPS coordinates.

    GPS gives us latitude/longitude in degrees, but we need distance
    in meters. The haversine formula does this conversion, accounting
    for the Earth being a sphere.
    """
    R = 6371000  # Earth's radius in meters

    # Convert degrees to radians (math functions need radians)
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)

    a = (
        math.sin(dphi / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    )
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def total_path_distance(waypoints, order):
    """
    Calculate the total distance of flying waypoints in a given order.

    'order' is a list of indices, e.g. [0, 3, 1, 5, 2, 6, 4] means:
      fly to waypoint 0 first, then 3, then 1, then 5, ...

    We sum up the distance between each consecutive pair.
    """
    total = 0.0
    for i in range(len(order) - 1):
        wp_from = waypoints[order[i]]
        wp_to = waypoints[order[i + 1]]
        total += haversine_distance(
            wp_from["lat"], wp_from["lon"],
            wp_to["lat"], wp_to["lon"],
        )
    return total


def find_optimal_order(waypoints):
    """
    Try every possible ordering of waypoints and return the shortest one.

    Returns:
        best_order: list of indices in the optimal order
        best_distance: total distance in meters for that order
    """
    n = len(waypoints)
    indices = list(range(n))  # [0, 1, 2, 3, 4, 5, 6]

    best_order = None
    best_distance = float("inf")
    total_checked = 0

    # permutations([0,1,2,3,4,5,6]) gives us every possible ordering
    for perm in permutations(indices):
        dist = total_path_distance(waypoints, perm)
        total_checked += 1

        if dist < best_distance:
            best_distance = dist
            best_order = list(perm)

    return best_order, best_distance, total_checked


# -----------------------------------------------------------------------
# Run it
# -----------------------------------------------------------------------

if __name__ == "__main__":
    # Sample waypoints from last year's competition
    waypoints = [
        {"lat": 35.05987, "lon": -118.156, "alt": 15.24, "name": "WP_A"},
        {"lat": 35.05991, "lon": -118.152, "alt": 30.48, "name": "WP_B"},
        {"lat": 35.06121, "lon": -118.153, "alt": 22.86, "name": "WP_C"},
        {"lat": 35.06312, "lon": -118.155, "alt": 15.24, "name": "WP_D"},
        {"lat": 35.06127, "lon": -118.157, "alt": 15.24, "name": "WP_E"},
        {"lat": 35.06206, "lon": -118.159, "alt": 22.86, "name": "WP_F"},
        {"lat": 35.05989, "lon": -118.160, "alt": 30.48, "name": "WP_G"},
    ]

    print("=" * 60)
    print("WAYPOINT ORDER OPTIMIZER")
    print("=" * 60)
    print()

    # Show the waypoints
    print("Input waypoints:")
    for i, wp in enumerate(waypoints):
        print(f"  [{i}] {wp['name']}: ({wp['lat']:.5f}, {wp['lon']:.5f}), alt={wp['alt']:.1f}m")
    print()

    # Show the default (original) order distance
    default_order = list(range(len(waypoints)))
    default_distance = total_path_distance(waypoints, default_order)
    print(f"Default order (A→B→C→D→E→F→G): {default_distance:.1f} m")
    print()

    # Find the best order
    print("Trying all 5040 orderings...")
    best_order, best_distance, total_checked = find_optimal_order(waypoints)

    print(f"Checked {total_checked} orderings.")
    print()
    print(f"BEST ORDER: {' → '.join(waypoints[i]['name'] for i in best_order)}")
    print(f"BEST DISTANCE: {best_distance:.1f} m")
    print(f"SAVINGS vs default: {default_distance - best_distance:.1f} m "
          f"({(1 - best_distance / default_distance) * 100:.1f}%)")
    print()

    # Show the leg-by-leg breakdown
    print("Leg-by-leg breakdown:")
    for i in range(len(best_order) - 1):
        wp_from = waypoints[best_order[i]]
        wp_to = waypoints[best_order[i + 1]]
        leg_dist = haversine_distance(
            wp_from["lat"], wp_from["lon"],
            wp_to["lat"], wp_to["lon"],
        )
        print(f"  {wp_from['name']} → {wp_to['name']}: {leg_dist:.1f} m")
