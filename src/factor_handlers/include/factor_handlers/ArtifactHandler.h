#ifndef ARTIFACT_HANDLER_H
#define ARTIFACT_HANDLER_H

// Includes
// For common datastructures
#include "utils/CommonStructs.h"
#include <core_msgs/Artifact.h>
#include <tf2_ros/transform_listener.h>

// Base class
#include "LampDataHandlerBase.h"

namespace pu = parameter_utils;
namespace gu = geometry_utils;

/*! \brief Stores Artifact information
 */
struct ArtifactInfo {
  std::string id;                       // this corresponds to parent_id
  int num_updates;                      // how many times the optimizer has updated this
  gtsam::Point3 global_position;        // Global pose of the artifact
  core_msgs::Artifact msg;              // All fields in the artifact message that we need
  ArtifactInfo(std::string art_id = "")
    : id(art_id), num_updates(0), global_position(gtsam::Point3()) {}
};

/*! \brief  Handles artifact messages. Takes artifact data from the artifact
 message -
          - Timestamp of the artifact message to help decide where to add the
 artifact in the pose graph
          - Artifact relative transformation to make factor in pose graph
          - Key of the last corresponding artifact node.
 * \input   Artifact message
 * \output  Current Timestamp, key of last corresponding artifact node and
 relative transform to the pose graph
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
    virtual bool Initialize(const ros::NodeHandle& n);
    
    /*! \brief  Gives the artifact associated data to the caller.
     * Returns  Artifact data
     */
    virtual FactorData* GetData();

    /*! \brief  Get the artifact_key2info_hash_
     * Returns  artifact_key2info_hash_
     */
    std::unordered_map<long unsigned int, ArtifactInfo>& GetArtifactKey2InfoHash() {return artifact_key2info_hash_;};

    /*! \brief  Updates the global pose of an artifact 
     * Returns  bool
     */
    bool UpdateGlobalPosition(const gtsam::Symbol artifact_key ,const gtsam::Point3 global_position);

    /*! \brief  Publish Artifact
     * Returns  Void
     */
    void PublishArtifacts(const gtsam::Symbol artifact_key ,const gtsam::Pose3 global_pose);

    protected:
    /*! \brief Load artifact parameters. 
     * n - Nodehandle
     * Returns bool
     */
    virtual bool LoadParameters(const ros::NodeHandle& n);

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
    virtual bool RegisterOnlineCallbacks(const ros::NodeHandle& n);

    /*! \brief Compute transform from Artifact message.
     * Not sure how necessary this is ????
     * Returns Transform
     */
    Eigen::Vector3d ComputeTransform(const core_msgs::Artifact& msg) const;

    /*! \brief  Get artifacts ID from artifact key
     * Returns Artifacts ID
     */
    std::string GetArtifactID(const gtsam::Symbol artifact_key) const;

    /*! \brief  Callback for Artifacts.
     * Returns  Void
     */
    void ArtifactCallback(const core_msgs::Artifact& msg);
    
    /*! \brief  Create publisher for the artifacts.
     * Returns  Void
     */
    virtual bool CreatePublishers(const ros::NodeHandle& n);

    /*! \brief  Print Artifact input message for debugging
     * Returns  Void
     */
    void PrintArtifactInputMessage(const core_msgs::Artifact& msg) const;

    /*! \brief  Extracts covariance from artifact message and converts to gtsam::SharedNoiseModel
     * Returns  gtsam::SharedNoiseModel
     */
    gtsam::SharedNoiseModel ExtractCovariance(const boost::array<float, 9> covariance) const;

    /*! \brief  Clear artifact data
     * Returns  Void
     */
    void ClearArtifactData();

    /*! \brief  Add artifact data
     * Returns  Void
     */
    virtual void AddArtifactData(const gtsam::Symbol artifact_key, const ros::Time time_stamp, const gtsam::Point3 transform, const gtsam::SharedNoiseModel noise);

    /*! \brief  Stores/Updated artifactInfo Hash
     * Returns  Void
     */
    void StoreArtifactInfo(const gtsam::Symbol artifact_key, const core_msgs::Artifact& msg);

    // Stores the artifact id to info mapping which is used to update any artifact associated parameters 
    // from the pose graph
    std::unordered_map<long unsigned int, ArtifactInfo> artifact_key2info_hash_;

    // Mapping between a artifact id and the node where it is present in the pose graph
    // TODO: Make keys as symbols gtsam.
    std::unordered_map<std::string, gtsam::Symbol> artifact_id2key_hash;

    
    // Parameters
    bool b_artifacts_in_global_;
    int largest_artifact_id_; 
    bool use_artifact_loop_closure_;

    // Namespace for publishing
    std::string name_;

    // Transformer
    tf2_ros::Buffer tf_buffer_;

    // Publisher
    ros::Publisher artifact_pub_;

    // Subscribers
    ros::Subscriber artifact_sub_;

    // Artifact prefix
    unsigned char artifact_prefix_;
    
    private:

    // Artifact output data
    ArtifactData artifact_data_;
    
    // Test class
    friend class TestArtifactHandler;
};

#endif // !ARTIFACT_HANDLER_H