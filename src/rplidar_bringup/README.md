# Package for further development with the Rplidar

A ros package `rplidar_ros` is provided for basic device handling with the laser scanner Rplidar. See [here] for more details (http://wiki.ros.org/rplidar) on e.g. parameter settings and message interfaces. 

You can use this packge as a basis for further development using lidar technology, such as lidar based perception or mapping and localization. 

## Lauching the lidar module

    % roslaunch rplidar_bringup rplidar_bringup.launch

## Hint for further development in mapping and localization

ROS has implementation for laser-based SLAM and the Adaptive Monte Carlo Localization algorithm. Check the link for [gmapping](http://wiki.ros.org/gmapping) and [acml](https://wiki.ros.org/amcl) respectively. 