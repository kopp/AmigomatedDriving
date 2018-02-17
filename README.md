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
workspace.  For the latter case see the Quick Start below.

Note: The `.gitignore` already ignores the build/devel/... folders for a
default catkin workspace layout.


# Quick Start

Tested with Ubuntu 14.04 and 16.04.1.

1. Install ros [as described here](http://wiki.ros.org/ROS/Installation).
    Note: Do *not* follow the instructions
    [here](http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA)
    because they install packages using `dpkg` (we only want to use `apt-get` or similar).
    Make sure, that you have installed `python-catkin-tools` as well.
1. Install all dependencies for this software

        sudo rosdep init # if you have not done this before
        git clone --branch kinetic-devel https://github.com/pal-robotics/aruco_ros.git src/external/aruco_ros # for ubuntu 16.04 only
        rosdep update
        sudo apt-get update # or equivalent
        rosdep install --from-paths ./src --ignore-src --default-yes
1. Use this workspace:

        catkin init
        catkin build
        source devel/setup.bash
1. Connect to the robot, see information in the `util`, e.g.

        rosrun rosaria RosAria _port:=10.0.126.14:8101

    and play with it

        rostopic pub -1 /RosAria/cmd_vel geometry_msgs/Twist '[0.3, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

    See [here](http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA)
    for more introductory stuff.


Alternatively see [here](https://github.com/kopp/ros-dockers) for how to
build a docker image with the contents of this package.

*Note* You will need to source the `devel/setup.bash` every time you
open a new terminal/start a new bashshell.


# README.md

Please include a `README.md` in each catkin package and other folders which
describes, what the purpose of the folder is/how it is organized etc.
