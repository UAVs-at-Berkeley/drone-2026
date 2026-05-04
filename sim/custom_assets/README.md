# Custom Gazebo Assets

This folder is the source-controlled home for custom Gazebo models used by the web SITL container.

## Folder layout

- `models/custom_target/model.config`
- `models/custom_target/model.sdf`
- `models/custom_target/meshes/` (put your Blender export here, for example `target.obj`)
- `models/sim_gimbal_camera/model.config`
- `models/sim_gimbal_camera/model.sdf` (camera body + Gazebo camera sensor used by sim backend)

## Replacing starter geometry with your Blender mesh

1. Export your model from Blender (recommended: `OBJ` + `MTL` + texture images).
2. Copy files into `models/custom_target/meshes/`.
3. Edit `models/custom_target/model.sdf` and replace the starter `<box>` geometry with:

```xml
<mesh>
  <uri>model://custom_target/meshes/target.obj</uri>
  <scale>1 1 1</scale>
</mesh>
```

Use the same `<mesh>` block in both collision and visual elements.

## Randomized placement

The container startup can spawn one or more target models at random positions each run.
Configure with environment variables in `sim/docker/docker-compose.yml`:

- `ENABLE_RANDOM_TARGET` (`1` to enable, default)
- `TARGET_MODEL_LIST` (comma-separated models to spawn, each with independent random placement)
- `TARGET_RANDOM_RADIUS_M` (max radius from origin in meters)
- `TARGET_MIN_RADIUS_M` (min radius from origin in meters)
- `TARGET_Z_M` (spawn height in meters)
- `TARGET_RANDOM_YAW` (`1` random yaw, `0` fixed yaw)

By default, web SITL uses:

- `custom_target`
- `custom_target_2`
- `targetNumber1` through `targetNumber10`

Each model in `TARGET_MODEL_LIST` is spawned by the same annulus-uniform heuristic (same logic as
`spawn_random_target.sh` for the original target), so every target gets independent randomized
position and yaw on startup.

`TARGET_MODEL_NAME` / `TARGET_SDF_PATH` are still supported as single-target fallback env vars.

## Sim gimbal camera model

The web SITL startup can also spawn `sim_gimbal_camera` automatically. The ROS `camera_node`
sim backend tracks the drone position and uses Gazebo `set_pose` to keep this camera model mounted
to the aircraft while holding commanded gimbal orientation until a new `/camera/move` goal arrives.
