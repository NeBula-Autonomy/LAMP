#ifndef ARTIFACT_HANDLER_H
#define ARTIFACT_HANDLER_H

/*! \brief Stores Artifact information
 */
struct ArtifactInfo {
  std::string id;                   // this corresponds to parent_id
  core_msgs::Artifact msg;          // All fields in the artifact message that we need
  int num_updates;                  // how many times the optimizer has updated this
  ArtifactInfo(std::string art_id="") :
               id(art_id), 
               num_updates(0){}
};

/*! \brief  Handles artifact messages. 
 * \input   Artifact message
 * \output  Keys and transform to the pose graph
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

    /*! \brief Load artifact parameters. 
     * n - Nodehandle
     * Returns bool
     */
    bool LoadParameters(const ros::NodeHandle& n);

    /*! \brief Register callbacks. 
     * n - Nodehandle
     * Returns bool
     */
    bool RegisterArtifactsCallback(const ros::NodeHandle& n);

    /*! \brief Compute transform from Artifact message.
     * Returns Transform
     */
    void ComputeTransform();

    /*! \brief  Get artifacts id and if not create one.
     * Returns Artifacts Id
     */
    void GetArtifactID();

    /*! \brief  Get this artifacts last observed node from map.
     * Returns Last observed Key of this Artifact 
     */
    void GetLastObservedArtifactKey();

    /*! \brief  Checks if artifact is a new one.
     * Returns  True if new or false otherwise 
     */
    bool IsNewArtifact();

    /*! \brief  Gives the factors to be added.
     * Returns  Factors 
     */
    bool GetFactors();

    /*! \brief  Gives the values.
     * Returns  Values 
     */
    bool GetValues();

    /*! \brief  Callback for Artifacts.
     * Returns  Void
     */
    void ArtifactCallback(const core_msgs::Artifact& msg);
    
    /*! \brief  Callback for ArtifactBase.
     * Returns  Void
     */
    void BlamSlam::ArtifactBaseCallback(const core_msgs::Artifact::ConstPtr& msg);
    
    private:
    // Object IDs
    std::unordered_map<std::string, gtsam::Key> artifact_id2key_hash;
    std::uint64_t last_id_;
    
    // Parameters
    bool artifacts_in_global_;
    int largest_artifact_id_; 
    bool use_artifact_loop_closure_;

    // Artifact prefix
    unsigned char artifact_prefix_;

    // Namespace for publishing
    std::string name_;

    // Subscribers
    ros::Subscriber artifact_sub_;
    std::vector<ros::Subscriber> Subscriber_artifactList_;
}

#endif // !ARTIFACT_HANDLER_H