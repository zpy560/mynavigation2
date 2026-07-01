# nav2_bringup Launch Layout

## Structure

- Root launch files keep shared bringup helpers and compatibility wrappers.
- Scenario-specific launch flows are split into sibling bringup packages.
- Do not put scenario-specific world, map, or robot startup logic in shared root
  helper launch files unless all scenarios need it.

## Scenarios

- `myagv_test_bringup`: custom `locationpub` and `laserpub` test bringup.
- `myworld_bringup`: `myworld2` simulation with the `diffbot.sdf` model and
  matching map.
- `tb3_simulation_bringup`: TurtleBot3 Gazebo simulation using the default
  column map.

## Compatibility

- `myagv_test.launch.py`, `myworld.launch.py`, and
  `tb3_simulation_launch.py` in this directory are wrappers.
- Keep these wrappers so existing `ros2 launch nav2_bringup ...` commands remain
  valid after reorganizing scenario files.
