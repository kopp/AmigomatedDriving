# AmigomatedDriving

This repository should contain all files to run a
limited automated driving software on an AmigoBot,
controlled by a RaspberryPi3, using [Ros](ros.org)
with [RosAria](http://wiki.ros.org/ROSARIA).

For a very brief overview over ROS, see
[the ROS primer](RosPrimer.md).


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

Tested with Ubuntu 16.04 and ros kinetic.
If you use a VM with ros already installed, you can skip the first steps (see "**start here ...**" below).

1. Install ros [as described here](http://wiki.ros.org/ROS/Installation).
    Note: Do *not* follow the instructions
    [here](http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA)
    because they install packages using `dpkg` (we only want to use `apt-get` or similar).
    Make sure, that you have installed `python-catkin-tools` as well.
1. Initialize rosdep

        sudo rosdep init  # if you have not done this before
1. Install all dependencies for this software (**start here if you had ros already installed**)

        rosdep update
        sudo apt-get update  # or equivalent
        rosdep install --from-paths ./src --ignore-src --default-yes
1. Use this workspace:

        catkin init
        catkin build
        source devel/setup.bash  # you need to do this in every shell you want to use ros in
1. Connect to the robot, see information in the `util`, e.g.

        rosrun rosaria RosAria _port:=10.0.126.14:8101
        
    if the robot is connected to your PC via WiFi or

        rosrun rosaria RosAria _port:=/dev/ttyUSB0
        
    if it is connected by USB-serial-cable and play with it

        rostopic pub -1 /RosAria/cmd_vel geometry_msgs/Twist '[0.3, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

    See [here](http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA)
    for more introductory stuff.


Alternatively see [here](https://github.com/kopp/ros-dockers) for how to
build a docker image with the contents of this package.

*Note* You will need to source the `devel/setup.bash` every time you
open a new terminal/start a new bashshell.


# Hardware

See in
[`hardware_catalogue`](hardware_catalogue/Readme.md)
for an overview of available hardware and how to use it.


# Software

See in
[`software_catalogue`](software_catalogue/Readme.md)
for some small ideas how to use the hardware provided here with ROS.


# README.md

Please include a `README.md` in each catkin package and other folders which
describes, what the purpose of the folder is/how it is organized etc.

# LICENSE

All parts of the repository are under the license mentioned in the LICENSE.txt file as long as it is not noted differently.
