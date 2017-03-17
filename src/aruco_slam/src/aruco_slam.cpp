#include <iostream>
#include <cmath>

/* try to use the official stuff
#include "vertex_se2.h"
#include "vertex_point_xy.h"
#include "edge_se2.h"
#include "edge_se2_pointxy.h"
#include "types_tutorial_slam2d.h"
using namespace g2o::tutorial;
*/

#include "g2o/types/slam2d/types_slam2d.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "aruco_slam/bagfile_reader.h"
#include "aruco_slam/g2o_converter.h"

using namespace std;
using namespace g2o;

G2O_USE_TYPE_GROUP(slam2d);

int main()
{

  /*********************************************************************************
   * creating the optimization problem
   ********************************************************************************/
  aruco_slam::BagfileReader bagfile_reader("/data/2017-03-08-18-08_with_markers.bag", "/RosAria/pose", "/web_cam/aruco_marker_publisher/markers");
  bagfile_reader.extractData();

  ROS_INFO_STREAM("Extracted " << bagfile_reader.robot_poses.size() << " robot poses.");
  ROS_INFO_STREAM("Extracted " << bagfile_reader.landmark_positions.size() << " landmark positions.");
  ROS_INFO_STREAM("Extracted " << bagfile_reader.landmark_measurements.size() << " landmark measurements.");

  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  // allocating the optimizer
  SparseOptimizer optimizer;
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);

  optimizer.setAlgorithm(solver);

  aruco_slam::G2oConverter converter(optimizer, 0.0002, 1e-6, 0.0015);
  converter.addData(bagfile_reader);

  /*
  // adding the odometry to the optimizer
  // first adding all the vertices
  VertexSE2* robot = new VertexSE2;
  robot->setId(0);
  robot->setEstimate(SE2(0, 0, 0));
  optimizer.addVertex(robot);

  robot = new VertexSE2;
  robot->setId(1);
  robot->setEstimate(SE2(1, 0, 0));
  optimizer.addVertex(robot);

  // second add the odometry constraints
  EdgeSE2* odometry = new EdgeSE2;
  odometry->vertices()[0] = optimizer.vertex(0);
  odometry->vertices()[1] = optimizer.vertex(1);
  odometry->setMeasurement(SE2(0.7, 0, 0));
  //odometry->setInformation(
  optimizer.addEdge(odometry);
  */

  // add the landmark observations
  /*
  cerr << "Optimization: add " << simulator.landmarks().size() << " landmark vertices ... ";
  for (size_t i = 0; i < simulator.landmarks().size(); ++i) {
    const Simulator::Landmark& l = simulator.landmarks()[i];
    VertexPointXY* landmark = new VertexPointXY;
    robot->setId(p.id);
    landmark->setId(l.id);
    landmark->setEstimate(l.simulatedPose);
    optimizer.addVertex(landmark);
  }
  cerr << "done." << endl;

  cerr << "Optimization: add " << simulator.landmarkObservations().size() << " landmark observations ... ";
  for (size_t i = 0; i < simulator.landmarkObservations().size(); ++i) {
    const Simulator::LandmarkEdge& simEdge = simulator.landmarkObservations()[i];
    EdgeSE2PointXY* landmarkObservation =  new EdgeSE2PointXY;
    landmarkObservation->vertices()[0] = optimizer.vertex(simEdge.from);
    landmarkObservation->vertices()[1] = optimizer.vertex(simEdge.to);
    landmarkObservation->setMeasurement(simEdge.simulatorMeas);
    landmarkObservation->setInformation(simEdge.information);
    landmarkObservation->setParameterId(0, sensorOffset->id());
    optimizer.addEdge(landmarkObservation);
  }
  cerr << "done." << endl;
  */


  /*********************************************************************************
   * optimization
   ********************************************************************************/

  // dump initial state to the disk
  optimizer.save("tutorial_before.g2o");

  // prepare and run the optimization
  // fix the first robot pose to account for gauge freedom
  VertexSE2* firstRobotPose = dynamic_cast<VertexSE2*>(optimizer.vertex(0));
  firstRobotPose->setFixed(true);
  optimizer.setVerbose(true);

  cerr << "Optimizing" << endl;
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  cerr << "done." << endl;

  optimizer.save("tutorial_after.g2o");
  cerr << "output saved" << endl;

  // freeing the graph memory
  optimizer.clear();
  cerr << "optimizer cleared" << endl;

  // destroy all the singletons
  Factory::destroy();
  cerr << "factory destroyed" << endl;
  OptimizationAlgorithmFactory::destroy();
  cerr << "algorithm factory destroyed" << endl;
  HyperGraphActionLibrary::destroy();
  cerr << "hypergraph action lib destroyed" << endl;

  return 0;
}
