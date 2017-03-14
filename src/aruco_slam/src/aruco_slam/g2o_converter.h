#pragma once


// interface
#include <aruco_slam/bagfile_reader.h>
#include <g2o/core/sparse_optimizer.h>

// member
#include <vector>
#include <memory> // shared_ptr


namespace aruco_slam
{

/// convert data from BagfileReader to g2o data structures
class G2oConverter
{
private:

  /// optimizer to use to add the data to
  g2o::SparseOptimizer& optimizer_;

  /// memory management for robot pose vertices
  std::vector<std::unique_ptr<g2o::VertexSE2>> robot_pose_vertex_ptrs_;
  /// memory management for odometry edges
  std::vector<std::unique_ptr<g2o::EdgeSE2>> robot_odometry_edge_ptrs_;

  /// information matrix for odometry edges
  /// This assumes, that odometry comes every 0.1 s
  g2o::EdgeSE2::InformationType odometry_information_base;

  /// ids to use for vertices/edges
  int id_ = 0;

public:

  /// Pass Optimizer to which the data will be added.
  /// \param noise_x_m standard deviation in longitudinal direction in m for 0.1 s.
  G2oConverter(g2o::SparseOptimizer& optimizer, double noise_x_m, double noise_y_m, double noise_rot_rad)
    : optimizer_(optimizer)
  {
    g2o::EdgeSE2::InformationType covariance;
    covariance.fill(0.);
    covariance(0, 0) = noise_x_m * noise_x_m;
    covariance(1, 1) = noise_y_m * noise_y_m;
    covariance(2, 2) = noise_rot_rad * noise_rot_rad;
    odometry_information_base = covariance.inverse();
  }

  int newId()
  {
    return id_++;
  }

  bool addData(BagfileReader const & bagfile_data)
  {
    // data:
    /*
  std::vector<tf2::Stamped<g2o::SE2>> robot_poses;
  /// known positions of landmarks
  /// this should be relatively small -- only one for each physical landmark
  /// currently first position in odom
  std::vector<tf2::Stamped<g2o::Vector2D>> landmark_positions;
  /// Measurements of the landmarks relative to robot
  std::vector<tf2::Stamped<g2o::Vector2D>> landmark_measurements;
  */

    g2o::SE2 previous_pose;
    int id_previous_pose;
    bool is_previous_pose_available = false;
    for (auto const & pose : bagfile_data.robot_poses)
    {
      // robot pose
      /*
      robot_pose_vertex_ptrs_.emplace_back(new g2o::VertexSE2);
      g2o::VertexSE2* robot_pose = robot_pose_vertex_ptrs_.back().get();
      */
      g2o::VertexSE2* robot_pose = new g2o::VertexSE2;
      int pose_id = newId();
      robot_pose->setId(pose_id);
      robot_pose->setEstimate(pose);
      optimizer_.addVertex(robot_pose);


      // odometry increment as difference
      if (is_previous_pose_available)
      {
        /*
        robot_odometry_edge_ptrs_.emplace_back(new g2o::EdgeSE2);
        g2o::EdgeSE2* odometry = robot_odometry_edge_ptrs_.back().get();
        */
        g2o::EdgeSE2* odometry = new g2o::EdgeSE2;

        g2o::SE2 pose_increment = previous_pose.inverse() * pose;

        odometry->vertices()[0] = optimizer_.vertex(id_previous_pose);
        odometry->vertices()[1] = optimizer_.vertex(pose_id);
        odometry->setMeasurement(pose_increment);
        odometry->setInformation(odometry_information_base);

        optimizer_.addEdge(odometry);
      }

      // at the end, set previous pose
      previous_pose = pose;
      id_previous_pose = pose_id;
      is_previous_pose_available = true;
    }



  }


};


}
