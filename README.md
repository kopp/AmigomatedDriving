# AmigomatedDriving

This repository should contain all files to run a
limited automated driving software on an AmigoBot,
controlled by a [Udoo](), using [ROS](ros.org)
with [ROSAria](http://wiki.ros.org/ROSARIA).

For a very brief overview over ROS, see
[the ROS primer](ros_primer.md).

# set up this repo

Since some of the packages come from external git repositories and are included
as git submodule (folder `src/external/`) run

    git submodule update --init

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

    ```bash
    catkin init
    catkin build
    source devel/setup.bash  # you need to do this in every shell you want to use ros in
    ```

        rosrun rosaria RosAria _port:=10.0.126.14:8101
        
    if the robot is connected to your PC via WiFi or

        rosrun rosaria RosAria _port:=/dev/ttyUSB0
        
    if it is connected by USB-serial-cable and play with it

    rostopic pub -1 /RosAria/cmd_vel geometry_msgs/Twist '[0.3, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
    ```

> See [ROSARIA Wiki](http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA) for more introductory stuff.

Alternatively see [kopp/ros-dockers](https://github.com/kopp/ros-dockers) for how to
build a docker image with the contents of this package.

*Note* You will need to source the `devel/setup.bash` every time you
open a new terminal/start a new bashshell.

# Hardware

See in
[`hardware_catalogue`](hardware_catalogue/README.md)
for an overview of available hardware and how to use it.

# Software

See in
[`software_catalogue`](software_catalogue/README.md)
for some small ideas how to use the hardware provided here with ROS.

# README.md

Please include a `README.md` in each catkin package and other folders which
describes, what the purpose of the folder is/how it is organized etc.

# LICENSE

All parts of the repository are under the license mentioned in the [LICENSE](LICENSE.txt) file as long as it is not noted differently.

# Trouble Shooting

## `catkin build` produces `Could not create symlink`

It is possible, that the file system that you have your workspace in, does not support symbolic links.
This happens, if you have the workspace cloned on a windows folder and shared this folder with your linux in a VM.
A solution is to create a workspace somewhere else and just link the `src` folder into that (or move it)

		cd ~
        mkdir workspace
        cd workspace
        ln -s /media/share/amigomateddriving/src .
        catkin init
        catkin build

## `catkin build` produces `/usr/bin/env python\r: No such file or directory`

You have dos line encodings in some file.
Search for the offending string (e.g. `ag python` in the case above) to look for the file and then run `dos2unix <file>`.
