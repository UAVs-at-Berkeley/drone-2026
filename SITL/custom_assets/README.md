# Custom Gazebo Assets

This folder is the source-controlled home for custom Gazebo models used by the web SITL container.

## Folder layout

- `models/custom_target/model.config`
- `models/custom_target/model.sdf`
- `models/custom_target/meshes/` (put your Blender export here, for example `target.obj`)

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

The container startup can spawn `custom_target` at a random position each run.
Configure with environment variables in `SITL/web-sim/docker-compose.yml`:

- `ENABLE_RANDOM_TARGET` (`1` to enable, default)
- `TARGET_RANDOM_RADIUS_M` (max radius from origin in meters)
- `TARGET_MIN_RADIUS_M` (min radius from origin in meters)
- `TARGET_Z_M` (spawn height in meters)
- `TARGET_RANDOM_YAW` (`1` random yaw, `0` fixed yaw)
