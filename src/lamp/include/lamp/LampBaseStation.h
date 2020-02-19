/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#ifndef LAMP_BASE_STATION_H
#define LAMP_BASE_STATION_H

#include <lamp/LampBase.h>

#include <factor_handlers/ManualLoopClosureHandler.h>
#include <factor_handlers/PoseGraphHandler.h>
#include <factor_handlers/RobotPoseHandler.h>
#include <factor_handlers/ArtifactHandler.h>
#include <std_msgs/String.h>


// Services

// Class Definition
class LampBaseStation : public LampBase {
  public:
    // Constructor
    LampBaseStation();

    // Destructor
    ~LampBaseStation();    

    // Override base class functions where needed 
    virtual bool Initialize(const ros::NodeHandle& n, bool from_log);

  protected: 
    
    // Instantiate all handlers that are being used in the derived classes
    virtual bool InitializeHandlers(const ros::NodeHandle& n); 

    // Load parameters from yaml files
    virtual bool LoadParameters(const ros::NodeHandle& n);

    // Create subscribers
    virtual bool RegisterCallbacks(const ros::NodeHandle& n);

    // Create publishers
    virtual bool CreatePublishers(const ros::NodeHandle& n);

    // retrieve data from all handlers
    virtual bool CheckHandlers(); // - inside timed callback

    // Main update timer callback
    virtual void ProcessTimerCallback(const ros::TimerEvent& ev);

    // Callback for debugging - put any code inside this
    virtual void DebugCallback(const std_msgs::String msg);

    // Process keyed scan candidates to add to the map
    void AddKeyedScanCandidatesToMap();

    // Robots that the base station subscribes to
    std::vector<std::string> robot_names_;

    // Artifact ground truthing
    bool ProcessArtifactGT();
    std::vector<std::string> artifact_GT_strings_;
    std::vector<ArtifactGroundTruth> artifact_GT_;

    // Factor handler wrappers
    bool ProcessPoseGraphData(std::shared_ptr<FactorData> data);
    bool ProcessManualLoopClosureData(std::shared_ptr<FactorData> data);
    bool ProcessRobotPoseData(std::shared_ptr<FactorData> data);

    // Data handler classes
    PoseGraphHandler pose_graph_handler_;
    RobotPoseHandler robot_pose_handler_;

    // Subscribers
    ros::Subscriber debug_sub_;

    // Publishers
    std::map<char, ros::Publisher> publishers_pose_;
    ros::Publisher lamp_pgo_reset_pub_;


    // Booleans
    bool b_published_initial_node_;
    bool b_optimize_on_artifacts_;
    bool b_use_uwb_;


  private:
    // Overwrite base classs functions where needed

    // Data Handler classes
    ManualLoopClosureHandler manual_loop_closure_handler_; 

    // Track latest pose from each robot
    std::map<char, std::pair<gtsam::Key, gtsam::Pose3>> latest_node_pose_;

    // Vector list of keyed scans to add to map
    std::vector<gtsam::Symbol> keyed_scan_candidates_;

    // Test class fixtures
    friend class TestLampBase;
};


#endif
