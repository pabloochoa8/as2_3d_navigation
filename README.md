# as2_3d_navigation

External Aerostack2 package for 3D navigation experiments, plugins and validation scripts.

## Packages

- `as2_3d_map_interface`
- `as2_3d_map_server_plugin`
- `as2_3d_path_planner_plugin`
- `as2_3d_navigation_bringup`

## Validated test

The script `test_follow_path_3d.py` validates that `FollowPathBehavior` can execute a 3D path with variable altitude.

Tested in Gazebo using:

- ROS 2 Humble
- Aerostack2
- `project_sherec_navigation`
- drone namespace: `drone0`

## Run validation script

```bash
cd ~/Desktop/TFM/as2_3d_navigation

source /opt/ros/humble/setup.bash
source ~/Desktop/TFM/aerostack2/install/setup.bash
source ~/Desktop/TFM/as2_3d_navigation/install/setup.bash

python3 src/as2_3d_navigation_bringup/scripts/test_follow_path_3d.py --namespace drone0
