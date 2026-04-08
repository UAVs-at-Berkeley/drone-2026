# Mission YAML

Central Command loads a file whose path is set by the `mission_file` ROS parameter (see `bringup.launch.py`).

## Schema

```yaml
mission:
  steps:
    - step_id_string
    - { id: step_id, ... optional fields ... }
```

- `steps` is ordered. Central Command runs each step to completion (action success) before starting the next.
- Each entry is either a **string** (step id) or a **mapping** with required `id` and optional parameters.

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