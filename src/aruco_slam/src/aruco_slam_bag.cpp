#include <cmath> // TODO: Necessary?

// read bag files
#include "aruco_slam/bagfile_reader.h"
#include "aruco_slam/g2o_converter.h"

// g2o stuff
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"


// add g2o types
#include "g2o/types/slam2d/types_slam2d.h"
G2O_USE_TYPE_GROUP(slam2d);


#include "ros/ros.h" // logging

int main()
{
  // -- load data
  aruco_slam::BagfileReader bagfile_reader("/data/2017-03-08-18-08_with_markers.bag", "/RosAria/pose", "/web_cam/aruco_marker_publisher/markers");
  bagfile_reader.extractData();

  ROS_INFO_STREAM("Extracted " << bagfile_reader.robot_poses.size() << " robot poses.");
  ROS_INFO_STREAM("Extracted " << bagfile_reader.landmark_positions.size() << " landmark positions.");
  ROS_INFO_STREAM("Extracted " << bagfile_reader.landmark_measurements.size() << " landmark measurements.");

  // -- set up g2o
  using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
  using SlamLinearSolver = g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType>;

  g2o::SparseOptimizer optimizer;
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);

  ROS_INFO("Solver set up");

  // -- transfer data into g2o
  // parameters:
  // run through a maze for about 10 m yields a delta of 0.4 m (assumed to be 3 sigma)
  // typical velocity: 0.15 m/s --> time about 10 / 0.15 = 67 s
  // odometry is assumed to come every 0.1 s --> 670 odometry calls
  // noise in x direction: 0.4 / (3 * 670) m per odometry
  // in y direction: even lower
  // rotation: rotational error about pi/4
  aruco_slam::G2oConverter converter(optimizer, 0.0002, 1e-6, 0.0015);
  converter.addData(bagfile_reader);

  ROS_INFO("Data transferred to solver");

  // -- optimize

  // dump initial state to the disk
  optimizer.save("aruco_before.g2o");

  // prepare and run the optimization
  // fix the first robot pose to account for gauge freedom
  g2o::VertexSE2* firstRobotPose = dynamic_cast<g2o::VertexSE2*>(optimizer.vertex(0));
  assert(firstRobotPose != nullptr && "Unable to get first robot pose from optimizer");
  firstRobotPose->setFixed(true);
  optimizer.setVerbose(true);

  ROS_INFO("Initializing optimization");
  optimizer.initializeOptimization();
  ROS_INFO("Starting optimization");
  optimizer.optimize(10);
  ROS_INFO("Optimization done.");

  optimizer.save("aruco_after.g2o");

  // freeing the graph memory
  optimizer.clear();
  ROS_INFO("optimizer.clear()");

  // destroy all the singletons
  /*
  g2o::Factory::destroy();
  ROS_INFO("g2o::Factory::destroy()");
  g2o::OptimizationAlgorithmFactory::destroy();
  ROS_INFO("g2o::OptimizationAlgorithmFactory::destroy()");
  g2o::HyperGraphActionLibrary::destroy();
  ROS_INFO("g2o::HyperGraphActionLibrary::destroy()");
  */

  return 0;
}
