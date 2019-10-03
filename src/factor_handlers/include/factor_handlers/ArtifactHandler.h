#ifndef ARTIFACT_HANDLER_H
#define ARTIFACT_HANDLER_H

// Includes
// For common datastructures
#include "utils/CommonStructs.h"
#include <core_msgs/Artifact.h>

// Base class
#include "LampDataHandlerBase.h"

// TODO Namespaces. Needs to be removed once this goes into header of LampDataHadlerBase
#include <parameter_utils/ParameterUtils.h>
namespace pu = parameter_utils;
namespace gu = geometry_utils;

// TODO Move to CommonStructs
typedef geometry_utils::MatrixNxNBase<double, 3> Mat33;

/*! \brief Stores Artifact information
 */
struct ArtifactInfo {
  std::string           id;                   // this corresponds to parent_id
  int                   num_updates;          // how many times the optimizer has updated this
  gtsam::Pose3          global_pose;          // Global pose of the artifact
  ArtifactInfo(std::string art_id="") :
               id(art_id), 
               num_updates(0),
               global_pose(gtsam::Pose3())
               {}
};

/*! \brief  Handles artifact messages. Takes artifact data from the artifact message - 
          - Timestamp of the artifact message to help decide where to add the artifact in the pose graph
          - Artifact relative transformation to make factor in pose graph
          - Key of the last corresponding artifact node.
 * \input   Artifact message
 * \output  Current Timestamp, key of last corresponding artifact node and relative transform to the pose graph
 */
class ArtifactHandler : public LampDataHandlerBase {
    public:
    // Constructor
    ArtifactHandler();

    // Destructor
    virtual ~ArtifactHandler() = default;

    /*! \brief Initialize parameters and callbacks. 
     * n - Nodehandle
     * Returns bool
     */
    bool Initialize(const ros::NodeHandle& n);
    
    /*! \brief  Gives the artifact associated data to the caller.
     * Returns  Artifact data
     */
    void GetData(FactorData artifact_data);

    protected:
    /*! \brief Load artifact parameters. 
     * n - Nodehandle
     * Returns bool
     */
    bool LoadParameters(const ros::NodeHandle& n);

    /*! \brief Register callbacks. 
     * n - Nodehandle, from_log - ????
     * Returns bool
     */
    bool RegisterCallbacks(const ros::NodeHandle& n, bool from_log);

    /*! \brief Register Log callbacks. 
     * n - Nodehandle
     * Returns bool
     */
    bool RegisterLogCallbacks(const ros::NodeHandle& n);

    /*! \brief Register Online callbacks. 
     * n - Nodehandle
     * Returns bool
     */
    bool RegisterOnlineCallbacks(const ros::NodeHandle& n);

    /*! \brief Compute transform from Artifact message.
     * Not sure how necessary this is ????
     * Returns Transform
     */
    Eigen::Vector3d ComputeTransform(const core_msgs::Artifact& msg);

    /*! \brief  Get artifacts key and if not create one.
     * Returns Artifacts key
     */
    gtsam::Key GetArtifactKey(const core_msgs::Artifact& msg);


    /*! \brief  Checks if artifact is a new one.
     * Returns  True if new or false otherwise 
     */
    bool IsNewArtifact(const std::string artifact_id);

    /*! \brief  Callback for Artifacts.
     * Returns  Void
     */
    void ArtifactCallback(const core_msgs::Artifact& msg);
    
    /*! \brief  Create publisher for the artifacts.
     * Returns  Void
     */
    bool CreatePublishers(const ros::NodeHandle& n);

    /*! \brief  Updates the global pose of an artifact 
     * Returns  Void
     */
    void UpdateGlobalPose(gtsam::Key artifact_key ,gtsam::Pose3 global_pose);
    
    /*! \brief  TODO Incomplete
     * Returns  Void
     */
    void ArtifactBaseCallback(const core_msgs::Artifact::ConstPtr& msg);

    private:
    // Stores the artifact id to info mapping which is used to update any artifact associated parameters 
    // from the pose graph
    std::unordered_map<std::string, ArtifactInfo> artifact_id_to_info_;
    // Mapping between a artifact id and the node where it is present in the pose graph
    std::unordered_map<std::string, gtsam::Key> artifact_id2key_hash;

    
    // Parameters
    bool artifacts_in_global_;
    int largest_artifact_id_; 
    bool use_artifact_loop_closure_;

    // Artifact prefix
    unsigned char artifact_prefix_;

    // Namespace for publishing
    std::string name_;

    // Artifact output data
    FactorData artifact_data_;

    // Publisher

    // Subscribers
    ros::Subscriber artifact_sub_;
    std::vector<ros::Subscriber> Subscriber_artifactList_;
};

#endif // !ARTIFACT_HANDLER_H