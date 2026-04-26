# Mission YAML

Launch files load and validate the mission YAML from `mission_file`, then pass parsed mission/environment data as ROS parameters to mission nodes (including Central Command).

## Schema

```yaml
environment:
  Geofence:
    points:
      - [37.0, -122.0, 30.0]
  waypoints:
    points:
      - [37.0, -122.0, 30.0]
  red_target: [37.0, -122.0, 30.0]
  x_target: [37.0, -122.0, 30.0]
  number_target: [37.0, -122.0, 30.0]
mission:
  steps:
    - step_id_string
    - { id: step_id, ... optional fields ... }
```

- `environment` is required and sits at the same YAML level as `mission`.
- Coordinate format is positional: `[lat, long, alt_m]`.
- `Geofence.points` and `waypoints.points` are coordinate lists; each target field is one coordinate.
- `steps` is ordered. Central Command runs each step to completion (action success) before starting the next.
- Each entry is either a **string** (step id) or a **mapping** with required `id` and optional parameters.
- `central_command_node` no longer reads YAML files directly; it consumes launch-provided `mission_steps_json`.

## Allowed step ids


| id                    | Action                                                   | Notes                                                   |
| --------------------- | -------------------------------------------------------- | ------------------------------------------------------- |
| `takeoff`             | `OffboardTakeoff` (`offboard_takeoff`)                   | Optional `takeoff_altitude_m` (overrides node default). |
| `time_trial`          | `StartTimeTrial` (`/time_trial/start`)                   | Optional `placeholder` (uint8).                         |
| `object_localization` | `StartObjectLocalization` (`/object_localization/start`) | Optional `placeholder` (uint8).                         |
| `return_to_home`      | `ReturnToHome` (`return_to_home`)                        | Optional `custom_mode` (default `AUTO.RTL`).            |
| `land`                | `OffboardLand` (`offboard_land`)                         | Optional `min_pitch`, `yaw` (MAVROS `CommandTOL`).      |


## Example

See `example_mission.yaml` in this directory.