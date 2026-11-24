# circe_nav/maps

Saved occupancy grids go here (`<name>.yaml` + `<name>.pgm`).

Upstream's `navigation_launch.py` defaulted its map argument to
`/home/rover/rover_workspace/office_map.yaml` — a hardcoded path on a machine that is
not this one. That default is gone; maps resolve relative to this directory.

Save a map built by `slam.launch.py` with:

    ros2 run nav2_map_server map_saver_cli -f ~/circe_ws/src/circe_nav/maps/<name>

Note that CIRCE's Nav2 config expects `map -> odom` from slam_toolbox, not AMCL —
`config/nav2_params.yaml` has no `amcl:` block. A saved map is therefore for
slam_toolbox localization mode or for reference, not for an AMCL run, until that block
is added. See the header of `launch/navigation.launch.py`.
