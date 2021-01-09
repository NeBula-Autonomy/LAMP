/*
 * Copyright Notes
 *
 * Authors: Alex Stephens    (alex.stephens@jpl.nasa.gov)
 */

#ifndef ROBOT_POSE_HANDLER_H
#define ROBOT_POSE_HANDLER_H

// Includes 
#include <factor_handlers/LampDataHandlerBase.h>

namespace pu = parameter_utils;
namespace gu = geometry_utils;
namespace gr = geometry_utils::ros;

class RobotPoseHandler : public LampDataHandlerBase {

  public:

    RobotPoseHandler();
    virtual ~RobotPoseHandler();

    bool Initialize(const ros::NodeHandle& n, std::vector<std::string> robot_names);
    std::shared_ptr<FactorData> GetData() override;

  protected:

    // Node initialization.
    bool LoadParameters(const ros::NodeHandle& n);
    bool RegisterCallbacks(const ros::NodeHandle& n);
    bool CreatePublishers(const ros::NodeHandle& n);

    // Add a new robot to subscribe to during initialisation
    bool AddRobot(std::string robot);

    // Reset stored data
    void ResetPoseData();

    // Input callbacks
    // void PoseCallback(const geometry_msgs::PoseStamped& msg, std::string robot);
    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, std::string robot);


    // The node's name.
    std::string name_;

    // Publishers and subscribers
    std::vector<ros::Publisher> publishers_pose_;
    std::vector<ros::Subscriber> subscribers_pose_;

    // Pose graphs and keyed scans received from robot
    RobotPoseData data_;


    // Robots that the base station subscribes to
    std::set<std::string> robot_names_;


  private:

};

#endif
