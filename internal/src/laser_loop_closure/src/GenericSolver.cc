/* 
Generic solver class 
author: Yun Chang, Luca Carlone
*/

#include <generic_solver/GenericSolver.h>

GenericSolver::GenericSolver(int solvertype): 
  nfg_(gtsam::NonlinearFactorGraph()),
  values_(gtsam::Values()),
  solver_type_(solvertype) {

  std::cout << "instantiated generic solver." << std::endl; 
}


void GenericSolver::update(gtsam::NonlinearFactorGraph nfg, 
                           gtsam::Values values, 
                           gtsam::FactorIndices factorsToRemove) {
  // remove factors
  for (size_t index : factorsToRemove) {
    nfg_[index].reset();
  }

  // add new values and factors
  nfg_.add(nfg);
  values_.insert(values);
  bool do_optimize = true; 

  // print number of loop closures
  // std::cout << "number of loop closures so far: " << nfg_.size() - values_.size() << std::endl; 

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
    if (solver_type_ == 1) {
      gtsam::LevenbergMarquardtParams params;
      params.setVerbosityLM("SUMMARY");
      std::cout << "Running LM" << std::endl; 
      params.diagonalDamping = true; 
      values_ = gtsam::LevenbergMarquardtOptimizer(nfg_, values_, params).optimize();
    }else if (solver_type_ == 2) {
      gtsam::GaussNewtonParams params;
      params.setVerbosity("ERROR");
      std::cout << "Running GN" << std::endl; 
      values_ = gtsam::GaussNewtonOptimizer(nfg_, values_, params).optimize();
    }else if (solver_type_ == 3) {
      // something
    }
  }
}

void GenericSolver::robustUpdate(){
  // robustUpdate 
  // check if odometry (compare factor keys)
// if odometry
    // - update posesAndCovariances_odom_ (T(t-1).compose( T_odom ) -> added to trajectory to be T(t), together with covariance - for composition
    // you can use PY code such that you also compute the covariance )
    // - store factor in nfg_odom
  // - store latest pose in values_ (note: values_ is the optimized estimate, while trajectory is the odom estimate)

// if not odometry (loop closure) - in this case we should run the pairwise consistency check to see if loop closure is good
  // * odometric consistency check (will only compare against odometry - if loop fails this, we can just drop it)
  // -- assume loop is between pose i and j
  // -- access (T_i,Cov_i) and (T_j, Cov_j) from trajectory_
  // -- compute Tij_odom = T_i.between(T_j); compute Covij_odom = Cov_j - Cov_i (Yun: verify if true) 
  // -- call pairwise check: isOdomConsistent = (Tij_odom,Cov_ij_odom, Tij_lc, Cov_ij_lc);
  // if (isOdomConsistent)
  // -- add factor to nfg_lc_
  // else 
  // -- return

  // * pairwise consistency check (will also compare other loops - if loop fails we still store it, but not include in the optimization)
  // -- add 1 row and 1 column to lc_adjacency_matrix_;
  // -- populate extra row and column by testing pairwise consistency of new lc against all previous ones
  // -- compute max clique
  // -- add loops in max clique to a local variable nfg_good_lc

  // * optimize and update values
  // NOTE: this will require a map from rowId (size_t, in adjacency matrix) to slot id (size_t, id of that lc in nfg_lc)
}

// read book + preliminary on preintegration 