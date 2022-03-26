/*
 * Copyright Notes
 *
 * Authors: Alex Stephens    (alex.stephens@jpl.nasa.gov)
 */

#ifndef POSE_GRAPH_HANDLER_H
#define POSE_GRAPH_HANDLER_H

// Includes
#include <factor_handlers/LampDataHandlerBase.h>
#include <unordered_map>
#include <unordered_set>
#include <lamp_utils/PointCloudUtils.h>

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

    std::unordered_set<uint64_t> keyed_scans_keys_;
    std::unordered_map<unsigned char,uint64_t> last_keyed_scan_key_from_robot_;
    std::unordered_set<uint64_t> pose_graph_node_keys_;
    std::unordered_map<unsigned char,uint64_t> last_odom_node_key_from_robot_;

    // Parameters when recomputing normals for republishing keyed scans on base
    lamp_utils::NormalComputeParams normals_compute_params_;

  private:

};

#endif
