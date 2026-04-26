# Mission YAML

Launch files load and validate the mission YAML from `mission_file`, then pass parsed mission/environment data as ROS parameters to mission nodes (including Central Command).

## Schema

Default waypoints style (see `example_mission.yaml` for a full file): a list of `[lat, long, alt_m]` under `environment.waypoints.points`. A plain `- time_trial` step reuses that list for the time-trial path (via `mission_loader`).

```yaml
environment:
  Geofence:
    points:
      - [37.0, -122.0, 30.0]
  waypoints:
    points:
      - [35.05987, -118.156, 10.0]
      - [35.05991, -118.152, 30.0]
  red_target: [37.0, -122.0, 30.0]
  x_target: [37.0, -122.0, 30.0]
  number_target: [37.0, -122.0, 30.0]
mission:
  steps:
    - takeoff
    - time_trial
    - step_id_string
    - { id: step_id, ... optional fields ... }
```

- `environment` is required and sits at the same YAML level as `mission`.
- Coordinate format is positional: `[lat, long, alt_m]`.
- `Geofence.points` and `waypoints.points` are coordinate lists; each target field is one coordinate.
- `steps` is ordered. Central Command runs each step to completion (action success) before starting the next.
- Each entry is either a **string** (step id) or a **mapping** with required `id` and optional parameters.
- For `time_trial`, define the path only in `environment.waypoints.points` as `[lat, long, alt_m]` triples; use a plain `- time_trial` step (do not put `latitudes` / `longitudes` / `altitudes` on the step). `mission_loader` copies that list into the action goal for Central Command.
- `central_command_node` loads steps only from `mission_file` (absolute path on disk). Launch passes that path from the `mission_file` launch argument.
- `time_trial_node` does not read waypoints from parameters; it uses `StartTimeTrial` goals (built from the mission by Central Command and `mission_loader`).

## Allowed step ids


| id                    | Action                                                   | Notes                                                   |
| --------------------- | -------------------------------------------------------- | ------------------------------------------------------- |
| `takeoff`             | `OffboardTakeoff` (`offboard_takeoff`)                   | Optional `takeoff_altitude_m` (overrides node default). |
| `time_trial`          | `StartTimeTrial` (`/time_trial/start`)                   | Path from `environment.waypoints.points` only (see above). |
| `object_localization` | `StartObjectLocalization` (`/object_localization/start`) | Optional `placeholder` (uint8).                         |
| `return_to_home`      | `ReturnToHome` (`return_to_home`)                        | Optional `custom_mode` (default `AUTO.RTL`).            |
| `land`                | `OffboardLand` (`offboard_land`)                         | Optional `min_pitch`, `yaw` (MAVROS `CommandTOL`).      |


## Example

See `example_mission.yaml` in this directory.