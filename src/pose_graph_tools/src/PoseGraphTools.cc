#include "pose_graph_tools/PoseGraphTools.h"

namespace pose_graph_tools
{

PoseGraphToolsNode::PoseGraphToolsNode(ros::NodeHandle &nh, dynamic_reconfigure::Server<Config> &dsrv) :
node_candidate_key_(0)
{
  // Dynamic Reconfigure
  dynamic_reconfigure::Server<Config>::CallbackType dsrv_cb;
  dsrv_cb = boost::bind(&PoseGraphToolsNode::DynRecCallback, this, _1, _2);
  dsrv.setCallback(dsrv_cb);

  //init class attributes if necessary

  // Get launch parameters
  //double d_example;
  //nh.param<double>("d_example",d_example,0.5);

  // [init publishers]
  this->pose_graph_out_publisher_ = nh.advertise<pose_graph_msgs::PoseGraph>("pose_graph_out", 1);
  
  // [init subscribers]
  this->pose_graph_in_subscriber_ = nh.subscribe("pose_graph_in", 1, &PoseGraphToolsNode::pose_graph_in_callback, this);
  pthread_mutex_init(&this->pose_graph_in_mutex_,NULL);

  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

PoseGraphToolsNode::~PoseGraphToolsNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->pose_graph_in_mutex_);
}

void PoseGraphToolsNode::mainNodeThread(void)
{
  // [fill msg structures]

  if (this->config_.enable &&
      this->node_candidate_key_ != 0)
  {
    // this->pose_graph_out_msg_.data = my_var;

    // this->pose_graph_out_publisher_.publish(this->pose_graph_out_msg_);
  }

  // Initialize the topic message structure
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
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

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void PoseGraphToolsNode::DynRecCallback(Config &config, uint32_t level)
{
  this->pgt_lib_.lock();

  if (!this->config_.enable && config.enable)
    this->pgt_lib_.print("PoseGraphToolsNode", " Enabled.", green);

  this->config_=config;
  this->pgt_lib_.unlock();
}

} //end of namespace