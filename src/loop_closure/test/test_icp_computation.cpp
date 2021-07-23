//
// Created by cdennist on 7/1/21.
//

#include <gtest/gtest.h>
#include <utils/PointCloudTypes.h>
#include "loop_closure/IcpLoopComputation.h"
#include <limits>
#include <chrono>
#include <thread>

class TestICPComputation : public ::testing::Test {
public:
    TestICPComputation()  {
        // Load params
        system("rosparam load $(rospack find "
               "loop_closure)/config/laser_parameters.yaml");
        system("rosparam load $(rospack find "
               "lamp)/config/precision_parameters.yaml");
        system("rosparam load $(rospack find lamp)/config/lamp_frames.yaml");
        system("rosparam load $(rospack find lamp)/config/lamp_rates.yaml");
        system("rosparam load $(rospack find lamp)/config/lamp_init_noise.yaml");
        system("rosparam load $(rospack find lamp)/config/lamp_settings.yaml");

        // Create data in the point cloud

    }

    lamp_loop_closure::IcpLoopComputation icp;
};
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
TEST_F(TestICPComputation, Initialization) {
    ros::NodeHandle nh("base"), pnh("base");

    bool result = icp.Initialize(nh);

    EXPECT_TRUE(result);
}

TEST_F(TestICPComputation, TestThreadedEqualsNonThreaded) {
    ros::NodeHandle nh("base"), pnh("base");
    ros::param::set("base/icp_thread_pool_thread_count", 1);
    ros::param::set("base/sac_ia/fitness_score_threshold",std::numeric_limits<double>::max());
    //Set the iterations really high so that we can get repeatable solutions
    ros::param::set("base/sac_ia/iterations",10000);
    ros::param::set("base/icp_lc/iterations",10000);
    bool result = icp.Initialize(nh);
    ASSERT_TRUE(result);

    size_t test_size = 100;

    //Make Keyed Scans
    std::vector<pose_graph_msgs::KeyedScan> scans;
    std::minstd_rand rng(0);
    std::uniform_real_distribution<float> distribution(0.1,0.05);
    pcl::PointCloud<pcl::PointXYZ> base_cloud;

    for (int x = 0; x < 10; x ++) {
        for (int y = 0; y < 10; y ++) {
            for (int z = 0; z < 10; z ++) {
                pcl::PointXYZ newPoint;
                newPoint.x = x * 0.001;
                newPoint.y = y * 0.001;
                newPoint.z = z * 0.001;
                base_cloud.points.push_back(newPoint);
            }
        }
    }
    for(gtsam::Key k=0; k < test_size; ++k){
        pose_graph_msgs::KeyedScan scan_msg;
        scan_msg.key = gtsam::Symbol('a', k);

        Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
        // create point cloud object
        pcl::PointCloud<pcl::PointXYZ> myCloud = base_cloud;

        transform_1(0,3) = (k % 2) == 1? -1.0 : 1.0;//distribution(rng);
        // Executing the transformation
        pcl::PointCloud<pcl::PointXYZ>transformed_cloud;
        // You can either apply transform_1 or transform_2; they are the same
        pcl::transformPointCloud (base_cloud, transformed_cloud, transform_1);

        sensor_msgs::PointCloud2 cloud;
        pcl::toROSMsg(transformed_cloud, cloud);
        scan_msg.scan = cloud;

        scans.push_back(scan_msg);
    }
    //Make Graph
    pose_graph_msgs::PoseGraph graph;
    for (gtsam::Key k=0; k < test_size; ++k) {
        pose_graph_msgs::PoseGraphNode node;
        node.key = gtsam::Symbol('a', k);
        node.ID = "odom_node";
        node.pose.position.x = k;
        node.pose.position.y = 0;
        node.pose.position.z = 0;
        graph.nodes.push_back(node);
    }
    pose_graph_msgs::LoopCandidateArray candidates;
    //Make Candidates
    for(gtsam::Key k = 0; k < test_size-1; ++k){
            pose_graph_msgs::LoopCandidate candidate;
            candidate.key_to = gtsam::Symbol('a', k);
            candidate.key_from = gtsam::Symbol('a', k + 1);
            candidate.pose_from.position.x = 0;
            candidate.pose_from.position.y = 0;
            candidate.pose_from.position.z = 0;
            candidate.pose_to.position.x = 0;
            candidate.pose_to.position.y = 0;
            candidate.pose_to.position.z = 0;
            candidates.candidates.emplace_back(candidate);
    }

    icp.KeyedPoseCallback(boost::make_shared<pose_graph_msgs::PoseGraph>(graph));
    for (auto scan : scans){
        icp.KeyedScanCallback(boost::make_shared<pose_graph_msgs::KeyedScan>(scan));
    }
    icp.InputCallback(boost::make_shared<pose_graph_msgs::LoopCandidateArray>(candidates));

    auto threadless_start = std::chrono::high_resolution_clock::now();
    icp.ComputeTransforms();
    auto threadless_end = std::chrono::high_resolution_clock::now();
    std::vector<pose_graph_msgs::PoseGraphEdge> threadless_output = icp.GetCurrentOutputQueue();

    std::chrono::seconds dura( 5);
    std::this_thread::sleep_for( dura );
    ros::param::set("base/icp_thread_pool_thread_count", .9);
    icp.PublishLoopClosures();
    result = icp.Initialize(nh);
    ASSERT_TRUE(result);
    icp.KeyedPoseCallback(boost::make_shared<pose_graph_msgs::PoseGraph>(graph));
    for (auto scan : scans){
        icp.KeyedScanCallback(boost::make_shared<pose_graph_msgs::KeyedScan>(scan));
    }
    icp.InputCallback(boost::make_shared<pose_graph_msgs::LoopCandidateArray>(candidates));

    auto threaded_start = std::chrono::high_resolution_clock::now();
    icp.ComputeTransforms();
    auto threaded_end = std::chrono::high_resolution_clock::now();

    double threadless_time = (threadless_end - threadless_start).count();
    double threadful_time = (threaded_end - threaded_start).count();
    double improvement = threadful_time / threadless_time;
    ROS_INFO_STREAM("Non-ThreadPool took " << threadless_time << ", Threaded took " << threadful_time << " Fractional Ratio: " << improvement);
    std::vector<pose_graph_msgs::PoseGraphEdge> threadful_output = icp.GetCurrentOutputQueue();


    EXPECT_EQ(threadful_output.size(),test_size-1);
    EXPECT_EQ(threadless_output.size(),test_size-1);

    for (auto threadless_closure : threadless_output){
        int matches = 0;
        for (auto threadful_closure : threadful_output){
            if ((threadless_closure.key_to == threadful_closure.key_to) && (threadless_closure.key_from == threadful_closure.key_from )){
                matches += 1;
                EXPECT_EQ(sgn(threadless_closure.pose.position.x), sgn(threadful_closure.pose.position.x));

            }
        }
        EXPECT_EQ(matches,1);
    }
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_icp_computation");
    ros::start();
    return RUN_ALL_TESTS();
}
