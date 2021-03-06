# Laserscanner RPLIDAR A2M8

![Laserscanner RPLIDAR A2M8](images/laserscanner_rplidar-a2m8.jpg)

## use with ros

    rosrun rplidar_ros rplidarNode

This assumes, that the package `rplidar_ros` was installed in the initial `rosdep install ...` run.

The RPLIDAR should start to spin.
This will by default publish to `/scan` (type `sensor_msgs/LaserScan`).

To test that the lidar is working, you can run `rviz` and enable plugin _LaserScan_.
Make sure to set the _Fixed Frame_ to `laser_frame`.

For a set of parameters see
[the documentation](http://wiki.ros.org/rplidar).

## Troubleshooting

In case you get something like

>  Error, cannot bind to the specified serial port /dev/ttyUSB0.

do `ls -hal /dev/ttyUSB0` to check the permissions of that device and add your user to the group indicated there, e.g.

    sudo usermod -a -G dialout john
    su john  # re-login to be in the group

if your name is `john` and the device is owned by `root:dialout`.

## Power Supply

The power source must be 5V DC ±2% with ripples of ~20 mV (max 50 mV).
It must provide up to 1.5A at 5VDC. But it is recommended to have a power supply able to provide 3A to sustain the peak at the start of the LIDAR.

For the connection, there is a barrel assembly
(e.g. in the [official store](https://www.robotshop.com/en/small-barrel-connector-assembly-07x235-mm-needs-soldering.html))
which provides a Power Barrel Connector Plug 0.70mm ID, 2.35mm OD EIAJ-1.
Simply connect the barrel (with soldering) to the 5VDC source; barrel interior is positive, outside negative.
