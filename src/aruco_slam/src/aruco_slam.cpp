
#include "aruco_slam/bagfile_reader.h"

#include "ros/ros.h" // logging

int main()
{
  aruco_slam::BagfileReader bagfile_reader("/data/2017-03-08-18-08_with_markers.bag", "/RosAria/pose", "/web_cam/aruco_marker_publisher/markers");
  bagfile_reader.extractData();

  ROS_INFO_STREAM("Extracted " << bagfile_reader.robot_poses.size() << " robot poses.");

}
