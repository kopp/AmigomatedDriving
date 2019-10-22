#!/bin/bash


AmigomatedDrivingFolderName="$HOME/base_AmigomatedDriving"
AmigomatedDrivingWeb="https://github.com/kopp/AmigomatedDriving"
AmigomatedDrivingGit="${AmigomatedDrivingWeb}.git"

# build catkin workspace?
build=false

# where to add a 'source' directive to source the catkin workspace
bashConfigFile="$HOME/.bashrc"

# add this to 'apt upgrade' and 'apt install' command lines
# set it to --assume-yes or --assume-no as you wish...
aptAssumeYes="--assume-yes"


function usage() {
    cat<<EOF
Setup a new Udoo x86 Ultra or Advanced Plus to be suitable as a controller for AmigomatedDriving.

See ${AmigomatedDrivingWeb}.

This script assumes, that it is executed on an Udoo, on which Ubuntu 16.04 has
been installed quite recently and which has access to the internet.

It will clone the AmigomatedDriving repository to ${AmigomatedDrivingFolderName}
and install all required dependencies.

Usage:
    $(basename $0) [--build]
Run this as normal user, not as root (or with sudo).

    --build     create catkin workspace in ${AmigomatedDrivingFolderName},
                build everything and add an entry in ${bashConfigFile}.
                Default: $build
EOF
}



# parse command line options
while [ -n "$1" ]
do
    case "$1" in
        (-h|--help)
            usage
            exit 0
            ;;
        (--build)
            build=true
            shift
            ;;
        (*)
            echo Error: Unknown option $1
            usage
            exit 1
            ;;
    esac
done



# actual program

if [ $(whoami) = root ]
then
    echo This script is not intended to be run as root.
    echo Run it as normal user.
    exit 1
fi


# make sure that the system is up to date
sudo apt update
sudo apt upgrade $aptAssumeYes

# remove/disable unattended upgrades
# the process often hangs and thus blocks other apt commands
sudo systemctl disable apt-daily.timer
sudo systemctl disable apt-daily-upgrade.timer


# install some required tools
sudo apt install $aptAssumeYes vim git openssh-server wget

# install the ros base system
# see http://wiki.ros.org/kinetic/Installation/Ubuntu
echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/ros-latest.list
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install $aptAssumeYes ros-kinetic-desktop-full python-catkin-tools

# initialize rosdep
sudo rosdep init
rosdep update

# automatically populate ROS environment variables in bash
echo '' >> "$bashConfigFile"
echo '# automatically populate ROS environment variables -- i.e. make ros usable by default' >> "$bashConfigFile"
echo "source /opt/ros/kinetic/setup.bash" >> "$bashConfigFile"

# make sure that the target folder to clone AmigomatedDriving is not already there
if [ -d "$AmigomatedDrivingFolderName" ]
then
    echo "Found folder $AmigomatedDrivingFolderName -- will rename that and clone new"
    newName="${AmigomatedDrivingFolderName}.bak.$(mktemp --dry-run XXXX)"
    mv -v "${AmigomatedDrivingFolderName}" "${newName}"
fi
# now clone the AmigomatedDriving repository
git clone --recursive $AmigomatedDrivingGit $AmigomatedDrivingFolderName
# install all dependencies
# Note: --rosdistro is necessary at this point, because
# /opt/ros/kinetic/setup.bash has not been sourced and thus ROS_DISTRO is not
# set.
rosdep install --rosdistro kinetic --from-paths "${AmigomatedDrivingFolderName}/src" --ignore-src --default-yes

# Note: Maybe it would be nice to disable splash on boot?

# if requested, make this a catkin workspace and build
if $build
then
    pushd "${AmigomatedDrivingFolderName}"
    catkin init
    catkin config --extend /opt/ros/kinetic
    catkin build
    config="${AmigomatedDrivingFolderName}/devel/setup.bash"
    source "$config"
    echo "# allow users to run AmigomatedDriving ros tools:" >> "$bashConfigFile"
    echo "source \"${config}\"" >> "$bashConfigFile"
    popd
else
    echo "Make sure to source the setup.bash files for each repo before using ros."
fi

