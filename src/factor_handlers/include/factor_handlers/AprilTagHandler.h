#ifndef APRIL_TAG_HANDLER_H_
#define APRIL_TAG_HANDLER_H_
// Includes
#include "factor_handlers/ArtifactHandler.h"

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

    /*! \brief Create publisher for April tag. 
     * n - Nodehandle
     * Returns bool
     */
    virtual bool CreatePublishers(const ros::NodeHandle& n);

    /*! \brief Register Online callbacks. 
     * n - Nodehandle
     * Returns bool
     */
    virtual bool RegisterOnlineCallbacks(const ros::NodeHandle& n); 

    /*! \brief  Callback for Artifacts.
     * Returns  Void
     */
    void AprilTagCallback(const core_msgs::Artifact& msg);

    private:
    // Test class
    friend class TestAprilTagHandler;
};

#endif  // APRIL_TAG_HANDLER_H_