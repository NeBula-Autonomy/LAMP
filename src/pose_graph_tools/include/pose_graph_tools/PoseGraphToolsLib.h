// Copyright (C)
// All rights reserved.
//
// This file is part of a free software package: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License as
// published by the Free Software Foundation, either version 3 of the License,
// or at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// IMPORTANT NOTE: This code has been generated through a script.
// Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts.

#ifndef _pose_graph_tools_lib_h_
#define _pose_graph_tools_lib_h_

#include <pose_graph_tools/PoseGraphToolsConfig.h>

#include <pose_graph_merger/merger.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphNode.h>

#include <pthread.h>

// Eigen
#include <Eigen/Dense>

// include pose_graph_tools_alg main library

namespace pose_graph_tools {

typedef enum {
  red = 0,
  green = 1,
  yellow = 2,
  blue = 3,
  magenta = 4,
  cyan = 5,
  white = 6
} color;

/**
 * \brief define config type
 *
 * Define a Config type with the PoseGraphToolsConfig.
 * All implementations will then use the same variable type Config.
 */
typedef pose_graph_tools::PoseGraphToolsConfig Config;

/**
 * \brief Eigen transform used in this class
 */
typedef Eigen::Transform<double, 3, Eigen::Affine> HTransf;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/**
 * \brief Mutex class
 *
 */
class CMutex {
 protected:
  /**
   * \brief define config type
   *
   * Define a Config type with the PoseGraphToolsConfig. All driver
   * implementations will then use the same variable type Config.
   */
  pthread_mutex_t access_;

 public:
  CMutex() { pthread_mutex_init(&this->access_, NULL); };
  ~CMutex() { pthread_mutex_destroy(&this->access_); };
  /**
   * \brief Lock Algorithm
   *
   * Locks access to the Algorithm class
   */
  void lock(void) { pthread_mutex_lock(&this->access_); };

  /**
   * \brief Unlock Algorithm
   *
   * Unlocks access to the Algorithm class
   */
  void unlock(void) { pthread_mutex_unlock(&this->access_); };

  /**
   * \brief Tries Access to Algorithm
   *
   * Tries access to Algorithm
   *
   * \return true if the lock was adquired, false otherwise
   */
  bool try_enter(void) {
    if (pthread_mutex_trylock(&this->access_) == 0)
      return true;
    else
      return false;
  };
};

/**
 * \brief Algorithm Class
 *
 *
 */
class PoseGraphToolsLib {
 protected:
  // private attributes and methods
  CMutex alg_mutex_;
  // Pose graph merger
  Merger merger_;

 public:
  /**
   * \brief config variable
   *
   * This variable has all the driver parameters defined in the cfg config
   * file. Is updated everytime function config_update() is called.
   */
  Config config_;

  /**
   * \brief constructor
   *
   * In this constructor parameters related to the specific driver can be
   * initalized. Those parameters can be also set in the openDriver() function.
   */
  PoseGraphToolsLib(void);

  /**
   * \brief Destructor
   *
   * This destructor is called when the object is about to be destroyed.
   *
   */
  ~PoseGraphToolsLib(void);

  /**
   * \brief Lock Algorithm
   *
   * Locks access to the Algorithm class
   */
  void lock(void) { this->alg_mutex_.lock(); };

  /**
   * \brief Unlock Algorithm
   *
   * Unlocks access to the Algorithm class
   */
  void unlock(void) { this->alg_mutex_.unlock(); };

  /**
   * \brief Tries Access to Algorithm
   *
   * Tries access to Algorithm
   *
   * \return true if the lock was adquired, false otherwise
   */
  bool try_enter(void) { return this->alg_mutex_.try_enter(); };

  // here define all pose_graph_tools_alg interface methods to retrieve and set
  // the driver parameters

  /**
   * \brief update node function
   * Given a delta_pose, and the node to apply to
   * Update the pose graph
   */
  pose_graph_msgs::PoseGraph updateNodePosition(
      const pose_graph_msgs::PoseGraph& msg,
      uint64_t key,
      HTransf delta_pose);
  /**
   * \brief update old corrected graph according to new graph
   */
  pose_graph_msgs::PoseGraph addNewIncomingGraph(
      const pose_graph_msgs::PoseGraphConstPtr& new_graph,
      const pose_graph_msgs::PoseGraphConstPtr& old_graph);

  /**
   * \brief Print RGB
   *
   * Print the specified string in the terminal
   */
  std::string bashColor(const color& c);
  std::string restartWhite(void);
  std::string print(const std::string& _name,
                    const std::string& _txt,
                    const color& _color);
  std::string print(const std::string& msg, const color& c);
  std::string print(const float& msg, const color& c);
};

}  // namespace pose_graph_tools

#endif
