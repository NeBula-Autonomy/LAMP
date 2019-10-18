#ifndef APRIL_TAG_HANDLER_H_
#define APRIL_TAG_HANDLER_H_
// Includes
#include "factor_handlers/ArtifactHandler.h"
#include <core_msgs/AprilTag.h>

class AprilTagHandler : public ArtifactHandler {
    public:
    // Constructor
    AprilTagHandler() {};
    // Destructor
    virtual ~AprilTagHandler() = default;

    /*! \brief Initialize parameters and callbacks. 
     * n - Nodehandle
     * Returns bool
     */
    virtual bool Initialize(const ros::NodeHandle& n);

    protected:

    /*! \brief Load April Tag parameters. 
     * n - Nodehandle
     * Returns bool
     */
    virtual bool LoadParameters(const ros::NodeHandle& n);

    /*! \brief Register Online callbacks. 
     * n - Nodehandle
     * Returns bool
     */
    virtual bool RegisterOnlineCallbacks(const ros::NodeHandle& n); 

    /*! \brief  Callback for April Tag.
     * Returns  Void
     */
    void AprilTagCallback(const core_msgs::AprilTag& msg);

    /*! \brief  Convert April tag message to Artifact message.
     * Returns  Artifacts message
     */
    core_msgs::Artifact ConvertAprilTagMsgToArtifactMsg(const core_msgs::AprilTag& msg);
    
    /*! \brief  Get ground truth data from April tag node key.  
     * Returns  Ground truth information
     */
    gtsam::Pose3 GetGroundTruthData(const gtsam::Symbol april_tag_key);
    
    private:
    // April related parameters
    // GT AprilTag world coordinates
    double calibration_left_x_;
    double calibration_left_y_;
    double calibration_left_z_;
    double calibration_right_x_;
    double calibration_right_y_;
    double calibration_right_z_;
    double distal_x_;
    double distal_y_;
    double distal_z_;
  
    // Test class
    friend class TestAprilTagHandler;
};

#endif  // APRIL_TAG_HANDLER_H_