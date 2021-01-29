/*
 * Copyright Notes
 *
 * Authors: Alex Stephens    (alex.stephens@jpl.nasa.gov)
 */

#ifndef POSE_GRAPH_HANDLER_H
#define POSE_GRAPH_HANDLER_H

// Includes 
#include <factor_handlers/LampDataHandlerBase.h>

namespace pu = parameter_utils;
namespace gu = geometry_utils;
namespace gr = geometry_utils::ros;

class PoseGraphHandler : public LampDataHandlerBase {

  public:

    PoseGraphHandler();
    virtual ~PoseGraphHandler();

    bool Initialize(const ros::NodeHandle& n, std::vector<std::string> robot_names);
    std::shared_ptr<FactorData> GetData() override;

  protected:

    // Node initialization.
    bool LoadParameters(const ros::NodeHandle& n);
    bool RegisterCallbacks(const ros::NodeHandle& n);
    bool CreatePublishers(const ros::NodeHandle& n);

    // Add a new robot to subscribe to during initialisation
    bool AddRobot(std::string robot);

    // Reset stored graph data
    void ResetGraphData();

    // Input callbacks
    void PoseGraphCallback(const pose_graph_msgs::PoseGraph::ConstPtr& msg);
    void KeyedScanCallback(const pose_graph_msgs::KeyedScan::ConstPtr& msg);

    // Publishers
    ros::Publisher keyed_scan_pub_;

    // The node's name.
    std::string name_;

    // Subscribers
    std::vector<ros::Subscriber> subscribers_posegraph;
    std::vector<ros::Subscriber> subscribers_keyedscan;

    // Pose graphs and keyed scans received from robot
    PoseGraphData data_;

    // Robots that the base station subscribes to
    std::set<std::string> robot_names_;


  private:

};

#endif
