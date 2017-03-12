#pragma once

// read from rosbag file
#include <rosbag/bag.h>
#include <rosbag/view.h>

// message types
#include <aruco_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>

// tf helper
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

// geometry helper
#include <Eigen/Geometry> // eulerAngles

// g2o types
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"

#include "g2o/types/slam2d/se2.h"
#include "g2o/core/eigen_types.h"

// member
#include <string>
#include <vector>
#include <set>
#include <memory> // unique_ptr

// logging
#include <ros/ros.h>

namespace aruco_slam
{

/// Read the required topics from bag file, transform to g2o type and store in vectors for easy access.
class BagfileReader
{
private:
  /// handle to the opened bag file
  rosbag::Bag bag_;
  /// view to iterate over all required messages
  /// pointer to set later in constructor
  /// Note: A single view for all topics is sufficient since all have different type
  std::unique_ptr<rosbag::View> view_;

  /// ids of known landmarks
  std::set<std::uint32_t> known_landmark_ids_;
  /// transform messages from the system
  tf2_ros::Buffer tf_buffer_;

  /// fixed frame
  std::string fixed_frame_;

public: // accessible data
  /// known poses of the robot
  std::vector<g2o::SE2> robot_poses;
  /// known positions of landmarks
  /// this should be relatively small -- only one for each physical landmark
  /// currently first position in odom
  std::vector<g2o::Vector2D> landmark_positions;

public:
  /// Open bag file to read with the required topics.
  /// \param bagfile_path path to the bag file to open
  /// \param robot_pose_topic topic name with the measured pose of the robot
  /// \param marker_pose_topic topic name with the measured marker pose
  /// \param fixed_frame name of the frame to assume fixed
  BagfileReader(std::string bagfile_path, std::string robot_pose_topic, std::string marker_pose_topic, std::string fixed_frame = "odom")
    : fixed_frame_(fixed_frame)
  {
    bag_.open(bagfile_path, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(robot_pose_topic);
    topics.push_back(marker_pose_topic);
    topics.push_back("/tf");

    view_.reset(new rosbag::View(bag_, rosbag::TopicQuery(topics)));

    // allow us to use canTransform
    // see http://answers.ros.org/question/205117/tf2-thread-setusingdedicatedthread-timeout/
    tf_buffer_.setUsingDedicatedThread(true);

  }

  ~BagfileReader()
  {
    bag_.close();
  }


  void extractData()
  {
    // get all static tf data
    rosbag::View tf_view(bag_, rosbag::TopicQuery(std::vector<std::string>{"/tf_static"}));
    for (rosbag::MessageInstance const m : tf_view)
    {
      auto tf_ptr = m.instantiate<tf2_msgs::TFMessage>();
      assert(tf_ptr != nullptr && "this loop should only handle tf messages");
      for (auto const & transform_msg : tf_ptr->transforms)
      {
        tf_buffer_.setTransform(transform_msg, "bag_file", true);
      }
    }

    // get all actual data
    for (rosbag::MessageInstance const m : *view_)
    {
      auto tf_ptr = m.instantiate<tf2_msgs::TFMessage>();
      if (tf_ptr != nullptr)
      {
        for (auto const & transform_msg : tf_ptr->transforms)
        {
          tf_buffer_.setTransform(transform_msg, "bag_file", false);
// TODO: Put new data to queue, then check here, whether the front can be transformed.  If so, do so and add to the appropriate data structures.
        }
        continue;
      }


      auto odom_ptr = m.instantiate<nav_msgs::Odometry>();
      if (odom_ptr != nullptr)
      {
        // this is a robot pose
        if(fixed_frame_.compare(odom_ptr->header.frame_id) != 0)
        {
          ROS_WARN_STREAM("Ignoring robot pose which is not in fixed frame " << fixed_frame_ << ": " << *odom_ptr);
        }
        else
        {
          Eigen::Affine3d eigen_pose;
          tf2::fromMsg(odom_ptr->pose.pose, eigen_pose);
          // get yaw
          Eigen::Vector3d euler_angles = eigen_pose.rotation().eulerAngles(0, 1, 2);
          double theta = euler_angles(2);
          robot_poses.emplace_back(odom_ptr->pose.pose.position.x, odom_ptr->pose.pose.position.y, theta);
        }
        continue;
      }

      auto markers_ptr = m.instantiate<aruco_msgs::MarkerArray>();
      if (markers_ptr != nullptr)
      {
        // iterate over each individual marker
        for (auto const & marker : markers_ptr->markers)
        {
          // check, whether this landmark is already known and only consider new landmarks
          if (known_landmark_ids_.find(marker.id) == known_landmark_ids_.end())
          {
            // landmark is not known
            // transform into fixed frame
            std::string error;
            if (tf_buffer_.canTransform(fixed_frame_, marker.header.frame_id, marker.header.stamp, ros::Duration(0), &error))
            {
              // ignore covariance as it is not filled anyways :-(
              // transform into fixed frame
              // transform works only with PoseStamped
              geometry_msgs::PoseStamped pose_input;
              pose_input.header = marker.header;
              pose_input.pose = marker.pose.pose;
              geometry_msgs::PoseStamped pose_fixed_frame;
              tf_buffer_.transform(pose_input, pose_fixed_frame, fixed_frame_, ros::Duration(0));
              landmark_positions.emplace_back(pose_fixed_frame.pose.position.x, pose_fixed_frame.pose.position.y);
            }
            else
            {
              // transformation would fail
              ROS_WARN_STREAM("Unable to transform landmark from " << marker.header.frame_id << " into fixed frame " << fixed_frame_ << " due to " << error);
            }
          }
        }
        continue;
      }
    }


  }

protected:

};

}
