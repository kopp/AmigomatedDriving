This package contains all files and tools that are needed to do mapping and navigation using an occupancy grid.


# launch

## amcl_diff_corrected.launch

Run localization using amcl (adaptive monte carlo localization) with the right model (diff-corrected).

## amigobot_2dnav.launch

Run localization (amcl_diff_corrected.launch) and navigation given a map (.pgm and .yaml) which is also loaded.

## slam_gmapping.launch

Mapping using an occupancy grid using parametrization which works quite well using packing boxes.
Drive around using:

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/RosAria/cmd_vel

When finished, save the map using

    rosrun map_server map saver -f MAP_NAME


