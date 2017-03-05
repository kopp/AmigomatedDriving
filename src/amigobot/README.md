This package contains all files and tools that are specific for the amigobot.


# launch

## publish_tfs.launch

Publish all required tf messages.
Launch this whenever you want to work with sensor data.

## display.launch

Show robot model in rviz (for debugging purposes).

## default_equipement_bringup.launch

Start all required nodes to bring up camera and sonar.
Use rviz config `rviz/default_equipement.rviz` to visualize the sensor data.



# Sonars

See the script `sonar_fov_visu` for a visualization of the field of view of the sonars.



# Move

You can use *teleop* to move the robot once some bringup was started
successfully.

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/RosAria/cmd_vel
