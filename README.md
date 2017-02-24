# AmigomatedDriving

This repository should contain all files to run a
limited automated driving software on an AmigoBot,
controlled by a RaspberryPi3, using [Ros](ros.org)
with [RosAria](http://wiki.ros.org/ROSARIA).



# set up this repo

Since some of the packages come from external git repositories and are included
as git submodule (folder `src/external/`) run

    git submodule init && git submodule update

after cloning this repository; see
[here](https://git-scm.com/book/en/v2/Git-Tools-Submodules)
for more info on submodules.


# work with packages: catkin workspace

Add the `src` folder to a catkin workspace (i.e. create another workspace and
link this `src` to the `src` there), or just create a workspace in the current
directory (`catkin init`), so that `src` is used as source path for the
workspace.

Note: The `.gitignore` already ignores the build/devel/... folders for a
default catkin workspace layout.


# Quick Start

1. Install ros and aria like
   [here](http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA).
   RosAria is included in this repo.
1. Use this workspace:

        catkin init && catkin build && source devel/setup.bash
1. Connect to the robot, see information in the `util`, e.g.

        rosrun rosaria RosAria _port:=10.0.126.14:8101

    and play with it

        rostopic pub -1 /RosAria/cmd_vel geometry_msgs/Twist '[0.3, 0.0, 0.0]' '[0.0, 0.0, 0.0]'


# README.md

Please include a `README.md` in each catkin package and other folders which
describes, what the purpose of the folder is/how it is organized etc.
