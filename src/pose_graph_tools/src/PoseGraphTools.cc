#include "pose_graph_tools/PoseGraphTools.h"

#include <utils/PrefixHandling.h>
#include <gtsam/inference/Symbol.h>

using Eigen::Translation;
using Eigen::AngleAxisd; 
using Eigen::Vector3d; 

namespace pose_graph_tools
{

PoseGraphToolsNode::PoseGraphToolsNode(ros::NodeHandle &nh, dynamic_reconfigure::Server<Config> &dsrv) :
node_candidate_key_(0)
{
  ROS_INFO("Initializing Pose Graph Tools");
  ROS_INFO("Pose Graph Tools: Initializing dynamic reconfigure");
  // Dynamic Reconfigure
  dynamic_reconfigure::Server<Config>::CallbackType dsrv_cb;
  dsrv_cb = boost::bind(&PoseGraphToolsNode::DynRecCallback, this, _1, _2);
  dsrv.setCallback(dsrv_cb);
  ROS_INFO("Pose Graph Tools: Setting publisher and subscriber");
  // [init publishers]
  this->pose_graph_out_publisher_ = nh.advertise<pose_graph_msgs::PoseGraph>("pose_graph_out", 1);
  
  // [init subscribers]
  this->pose_graph_in_subscriber_ = nh.subscribe("pose_graph_in", 1, &PoseGraphToolsNode::pose_graph_in_callback, this);
  pthread_mutex_init(&this->pose_graph_in_mutex_,NULL);
}

PoseGraphToolsNode::~PoseGraphToolsNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->pose_graph_in_mutex_);
}

void PoseGraphToolsNode::mainNodeThread(void) {
  if (this->config_.enable && this->node_candidate_key_ != 0) {
    this->pgt_lib_.lock();

    double dx = this->config_.dx;
    double dy = this->config_.dy;
    double dz = this->config_.dz;
    double droll = this->config_.droll;
    double dpitch = this->config_.dpitch;
    double dyaw = this->config_.dyaw;
    HTransf d_pose = Translation<double, 3>(dx, dy, dz) *
                     AngleAxisd(dyaw, Vector3d::UnitZ()) *
                     AngleAxisd(dpitch, Vector3d::UnitY()) *
                     AngleAxisd(droll, Vector3d::UnitX());
    std::cout << "Modifying key: "
              << gtsam::DefaultKeyFormatter(this->node_candidate_key_)
              << std::endl;
    std::cout << "With transform matrix: " << d_pose.matrix() << std::endl;
    this->pose_graph_out_msg_ = pgt_lib_.updateNodePosition(
        this->pose_graph_in_msg_, this->node_candidate_key_, d_pose);
    this->pgt_lib_.unlock();
  } 
  this->pose_graph_out_publisher_.publish(this->pose_graph_out_msg_);
}

/*  [subscriber callbacks] */
void PoseGraphToolsNode::pose_graph_in_callback(const pose_graph_msgs::PoseGraph::ConstPtr& msg)
{
  ROS_DEBUG("PoseGraphToolsNode::pose_graph_in_callback: New Message Received");
  this->pose_graph_in_mutex_enter();
  this->pose_graph_in_msg_ = *msg;
  this->pose_graph_in_mutex_exit();
}

void PoseGraphToolsNode::pose_graph_in_mutex_enter(void)
{
  pthread_mutex_lock(&this->pose_graph_in_mutex_);
}

void PoseGraphToolsNode::pose_graph_in_mutex_exit(void)
{
  pthread_mutex_unlock(&this->pose_graph_in_mutex_);
}

void PoseGraphToolsNode::DynRecCallback(Config& config, uint32_t level) {
  if (!this->config_.enable && config.enable) {
    this->pgt_lib_.print("PoseGraphToolsNode", " Enabled.", green);
  }

  this->config_ = config;

  if (config.enable) {
    std::string robot = config.robot;
    char prefix = utils::GetRobotPrefix(robot);
    int key = config.key;
    uint64_t candidate_key_ = gtsam::Symbol(prefix, key);
    if (candidate_key_ != this->node_candidate_key_) {
      ROS_INFO_STREAM("Reconfiguring key: "
                      << gtsam::DefaultKeyFormatter(this->node_candidate_key_));
      this->pose_graph_in_msg_ = this->pose_graph_out_msg_;
      this->node_candidate_key_ = candidate_key_;
    }
  }
}

} //end of namespace