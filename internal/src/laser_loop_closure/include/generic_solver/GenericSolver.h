/* 
Generic solver class 
author: Yun Chang, Luca Carlone
*/

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/inference/Symbol.h>

#include "graph_utils/graph_utils_functions.h" // from distriuted mapper

class GenericSolver {
public:
  GenericSolver();
  void update(gtsam::NonlinearFactorGraph nfg=gtsam::NonlinearFactorGraph(), 
              gtsam::Values values=gtsam::Values(),
              gtsam::FactorIndices factorsToRemove=gtsam::FactorIndices());

  gtsam::Values calculateEstimate() { return values_gs_; }
  gtsam::Values calculateBestEstimate() { return values_gs_; }
  gtsam::Values getLinearizationPoint() { return values_gs_; }
  gtsam::NonlinearFactorGraph getFactorsUnsafe(){ return nfg_gs_; }

  void print() {
    nfg_gs_.print("");
    values_gs_.print("");
  }

private:
  gtsam::Values values_gs_;
  gtsam::NonlinearFactorGraph nfg_gs_;
};