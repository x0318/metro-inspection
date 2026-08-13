# metro_description

ROS 2 description package for robot URDF files, meshes, collision geometry,
inertial properties, joints, and sensor mounting frames.

The active `subway_v2` geometry comes from
`feature/hardware:hardware/simple_v6` (commit `37ce04e`). The ROS 1 export is
normalized into this ROS 2 package; it is not installed as a second package.

Gazebo keeps the hardware URDF at native scale in this package, then generates
the runtime SDF at scale `3.0063795853269537`. The scale aligns the V6 tread
centers with the tunnel rail centers at `y = +/-0.754 m`. The DAE rail head is
about `0.050 m` wide; the scaled tread is about `0.0571 m` wide and the flange
retains about `3.6 mm` clearance from the rail inner face while centered.

The active robot description is installed as:

```text
share/metro_description/urdf/subway_v2.urdf
```

Gazebo runtime models are stored separately in:

```text
metro_sim/models/subway_v2/
metro_sim/models/subway_tunnel_v2/
```

The V6 CAD export contains the chassis, four wheels, yaw and pitch assemblies,
Odin1 housing, and four fixed-camera housings. Their CAD mounting transforms
are retained. Existing Odin1, IMU, and five camera ROS interfaces are preserved;
camera optical origins remain provisional until measured extrinsics are added.

In simple_v6, `base_link.STL` already contains the installed Odin1 housing and
`leida.STL` exports the same housing again. Runtime visual ownership therefore
belongs to `base_link`: the `leida` link retains one collision plus the TF and
sensor frames, but has no separate visual. `import_simple_v6.py` verifies that
at least 99% of the transformed `leida.STL` triangles are embedded in
`base_link.STL` before applying this rule. A future CAD export with different
ownership is rejected for review instead of silently hiding or duplicating a
model.

Both import and SDF generation are replacement operations, not append
operations. The importer reconstructs the URDF from the hardware source,
enforces unique Odin1 links, joints, sensors, and plugins, and keeps only the
runtime STL allowlist. SDF generation validates the same uniqueness, lidar
optical center and scan axis before replacing `model.sdf`; `rsync --delete`
removes meshes that are no longer part of the active model.

The full model keeps the lidar, IMU, and all six RGB cameras active so every
sensor topic is immediately available. The separate mapping launch derives a
temporary camera-free SDF from this same model and runs Gazebo without its GUI;
it does not copy meshes or change the real lidar's resolution, field of view,
range, noise, or requested frame rate.

Yaw and pitch are locked at the V6 CAD zero pose because this model does not yet
have a position controller for those joints. Making them freely revolute lets
gravity rotate the gimbal away from its mounting pose. They can be made movable
when a controller and commanded initial positions are added.

Each V6 wheel assembly is split reproducibly into a 520-triangle rotating
flanged wheel and a fixed motor/mount mesh. The wheel joint axes are moved to
the physical tread centers while the mesh origins are compensated, so the
zero-position CAD assembly is unchanged. Each rotating link owns two cylinder
collisions: a native radius `0.060 m`, length `0.019 m` tread and a native
radius `0.0775 m`, length `0.013 m` flange. The runtime values are uniformly
scaled with the vehicle.

`libgazebo_ros_diff_drive.so` drives all four wheel joints in two pairs and
keeps the established `/cmd_vel_drive`, `/odom`, and `odom -> base_footprint`
interfaces. The previous `planar_move` pose driver is not used by `subway_v2`;
the vehicle now advances through wheel joint motion and wheel/rail contact.
`tunnel_obstacle_guard.py` continues to remove lateral and yaw commands.
The CAD nose and Odin1 scan axis define `base_footprint +X`, so a positive
`linear.x` command moves toward the visible robot front. The importer validates
this convention and assigns wheels to the left/right drive pairs after applying
the CAD-to-REP-103 root rotation.

The V6 camera housings keep their CAD mounting translations, with inspection
orientations restored explicitly: `xj1` and `xj2` look outward to the tunnel
sides, while `xj3` and `xj4` look down toward the track. Camera body frames are
placed at the previously validated lens-center offsets instead of each STL's
link origin. The pitch camera frame is located on the camera geometry embedded
in `pitch.STL`; Odin1's calibrated internal transform is unchanged.

After changing the URDF or meshes, regenerate the Gazebo model from the
repository root:

```bash
./ros_ws/src/metro_sim/scripts/generate_subway_v2_sdf.sh
```
