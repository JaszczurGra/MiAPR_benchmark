# cuRobo robot config for UR5e

cuRobo needs a robot YAML (kinematics + collision spheres) to plan. It **ships one for
UR5e** at `curobo/src/curobo/content/configs/robot/ur5e.yml`, so the cuRobo adapter
loads `"ur5e.yml"` by name (resolved from cuRobo's content path) -- nothing to do here
for the default ur5e setup.

## If you want strict UR5 (CB-series), not ur5e

cuRobo does not ship a UR5 config. Generate one (do this inside the `curobo` container):

1. Get the UR5 URDF (from `ur_description`, xacro -> urdf with `ur_type:=ur5`).
2. Copy a ur5e.yml to `ur5.yml`, point `kinematics.urdf_path` at the UR5 URDF, fix
   `base_link` / `ee_link`.
3. Regenerate collision spheres for the UR5 geometry (cuRobo's robot-config / sphere
   tooling, or the Isaac Sim sphere generator). **Do not reuse ur5e spheres** -- wrong
   geometry makes cuRobo plan "collision-free" against the wrong body (a silent bug).
4. Pass `robot_config="ur5.yml"` to `CuRoboAdapter` and set `ur_type:=ur5` on the ROS
   side so both frameworks use the same robot.

This is why the project **defaults to ur5e everywhere** -- it is the validated,
testable choice. See `../../METHODOLOGY.md` (robot-fidelity decision).

Drop any custom `*.yml` you generate in this directory and mount it into the container.
