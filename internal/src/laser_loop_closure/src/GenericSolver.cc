/* 
Generic solver class 
author: Yun Chang, Luca Carlone
*/

#include <generic_solver/GenericSolver.h>

GenericSolver::GenericSolver(): 
  nfg_gs_(gtsam::NonlinearFactorGraph()),
  values_gs_(gtsam::Values()) {
  
  std::cout << "instantiated generic solver." << std::endl; 
}
// robustUpdate 
// TODOs: 
// move Generic Solver to other file
// 1. nfg_odom, 2. nfg_loop_closures, 3. values_odom 4. covariances_odom
// -computation: populate 1, 2 with factors 
// -update 3 and 4 whenever new odometry recieved + new artifact (redetection of artifact will be loop closure)
// - 3.4 should be a trajectory_odom (set of poses with corresponding covariance key to pose/covaraice see PY code )

void GenericSolver::update(gtsam::NonlinearFactorGraph nfg, 
                           gtsam::Values values, 
                           gtsam::FactorIndices factorsToRemove) {
  // remove factors
  for (size_t index : factorsToRemove) {
    nfg_gs_[index].reset();
  }

  // add new values and factors
  nfg_gs_.add(nfg);
  values_gs_.insert(values);
  bool do_optimize = true; 

  // print number of loop closures
  // std::cout << "number of loop closures so far: " << nfg_gs_.size() - values_gs_.size() << std::endl; 

  if (values.size() > 1) {ROS_WARN("Unexpected behavior: number of update poses greater than one.");}

  if (nfg.size() > 1) {ROS_WARN("Unexpected behavior: number of update factors greater than one.");}

  if (nfg.size() == 0 && values.size() > 0) {ROS_ERROR("Unexpected behavior: added values but no factors.");}

  // Do not optimize for just odometry additions 
  // odometry values would not have prefix 'l' unlike artifact values
  if (nfg.size() == 1 && values.size() == 1) {
    const gtsam::Symbol symb(values.keys()[0]); 
    if (symb.chr() != 'l') {do_optimize = false;}
  }

  // nothing added so no optimization 
  if (nfg.size() == 0 && values.size() == 0) {do_optimize = false;}

  if (factorsToRemove.size() > 0) 
    do_optimize = true;

  if (do_optimize) {
    ROS_INFO(">>>>>>>>>>>> Run Optimizer <<<<<<<<<<<<");
    // optimize
    #if SOLVER==1
    gtsam::LevenbergMarquardtParams params;
    params.setVerbosityLM("SUMMARY");
    std::cout << "Running LM" << std::endl; 
    params.diagonalDamping = true; 
    values_gs_ = gtsam::LevenbergMarquardtOptimizer(nfg_gs_, values_gs_, params).optimize();
    #elif SOLVER==2
    gtsam::GaussNewtonParams params;
    params.setVerbosity("ERROR");
    std::cout << "Running GN" << std::endl; 
    values_gs_ = gtsam::GaussNewtonOptimizer(nfg_gs_, values_gs_, params).optimize();
    #elif SOLVER==3
    // something
    #endif
  }
}