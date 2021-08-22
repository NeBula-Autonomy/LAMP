/**
 *  @brief Testing the Point Cloud Utils Functions
 *
 */

#include <gtest/gtest.h>

#include <math.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>

#include <utils/PointCloudUtils.h>

#include "test_artifacts.h"

namespace utils {

class TestPointCloudUtils : public ::testing::Test {
public:
  TestPointCloudUtils() {
    // Set params
  }
  ~TestPointCloudUtils() {}

protected:
  double tolerance_ = 1e-5;

  HarrisParams GenerateHarrisParams() {
    HarrisParams params;
    params.harris_response_ = 1;
    params.harris_threshold_ = 0.000000005;
    params.harris_suppression_ = true;
    params.harris_radius_ = 1.0;
    params.harris_refine_ = false;
    return params;
  }
};

TEST_F(TestPointCloudUtils, ComputeNormals) {
  PointCloud::Ptr plane(new PointCloud);
  Normals::Ptr plane_normals(new Normals);
  plane = GeneratePlane();
  NormalComputeParams default_params;
  ComputeNormals<Point>(plane, default_params, plane_normals);

  EXPECT_EQ(100, plane_normals->size());
  for (size_t i = 0; i < 100; i++) {
    EXPECT_NEAR(0, plane_normals->points[i].normal_x, tolerance_);
    EXPECT_NEAR(0, plane_normals->points[i].normal_y, tolerance_);
    EXPECT_NEAR(1, plane_normals->points[i].normal_z, tolerance_);
  }
}

TEST_F(TestPointCloudUtils, ExtractNormals) {
  PointCloud::Ptr plane(new PointCloud);
  Normals::Ptr plane_normals(new Normals);
  plane = GeneratePlane();
  ExtractNormals(plane, plane_normals);

  EXPECT_EQ(100, plane_normals->size());
  for (size_t i = 0; i < 100; i++) {
    EXPECT_NEAR(0, plane_normals->points[i].normal_x, tolerance_);
    EXPECT_NEAR(0, plane_normals->points[i].normal_y, tolerance_);
    EXPECT_NEAR(1, plane_normals->points[i].normal_z, tolerance_);
  }
}

TEST_F(TestPointCloudUtils, AddNormals) {
  PointXyziCloud::Ptr plane(new PointXyziCloud);
  PointCloud::Ptr plane_w_normals(new PointCloud);
  plane_w_normals = GeneratePlane();
  ConvertPointCloud(plane_w_normals, plane);
  plane_w_normals->clear();

  NormalComputeParams default_params;
  AddNormals(plane, default_params, plane_w_normals);

  EXPECT_EQ(100, plane_w_normals->size());
  for (size_t i = 0; i < 100; i++) {
    EXPECT_NEAR(0, plane_w_normals->points[i].normal_x, tolerance_);
    EXPECT_NEAR(0, plane_w_normals->points[i].normal_y, tolerance_);
    EXPECT_NEAR(1, plane_w_normals->points[i].normal_z, tolerance_);
  }
}

TEST_F(TestPointCloudUtils, normalizePCloud) {
  PointCloud::Ptr plane(new PointCloud);
  PointCloud::Ptr plane_normalized(new PointCloud);
  plane = GeneratePlane();
  NormalizePCloud(plane, plane_normalized);

  double sum_x = 0;
  double sum_y = 0;
  double sum_z = 0; // centroid
  double sum_of_dist = 0;
  for (size_t i = 0; i < 100; i++) {
    sum_x += plane_normalized->points[i].x;
    sum_y += plane_normalized->points[i].y;
    sum_z += plane_normalized->points[i].z;
    sum_of_dist +=
        sqrt((plane_normalized->points[i].x * plane_normalized->points[i].x) +
             (plane_normalized->points[i].y * plane_normalized->points[i].y) +
             (plane_normalized->points[i].z * plane_normalized->points[i].z));
  }

  EXPECT_EQ(100, plane_normalized->size());
  EXPECT_NEAR(0, sum_x / 100, tolerance_);
  EXPECT_NEAR(0, sum_y / 100, tolerance_);
  EXPECT_NEAR(0, sum_z / 100, tolerance_);
  EXPECT_NEAR(1, sum_of_dist / 100, tolerance_);
}

TEST_F(TestPointCloudUtils, ComputeKeypoints1) {
  PointCloud::Ptr corner(new PointCloud);
  PointCloud::Ptr corner_keypts(new PointCloud);
  corner = GenerateCorner();

  HarrisParams params = GenerateHarrisParams();

  ComputeKeypoints(corner, params, 4, corner_keypts);

  EXPECT_EQ(3, corner_keypts->size());
  EXPECT_NEAR(0.3, corner_keypts->points[0].x, tolerance_);
  EXPECT_NEAR(0.3, corner_keypts->points[0].y, tolerance_);
  EXPECT_NEAR(0, corner_keypts->points[0].z, tolerance_);
  EXPECT_NEAR(0.3, corner_keypts->points[1].x, tolerance_);
  EXPECT_NEAR(0, corner_keypts->points[1].y, tolerance_);
  EXPECT_NEAR(0.3, corner_keypts->points[1].z, tolerance_);
  EXPECT_NEAR(0, corner_keypts->points[2].x, tolerance_);
  EXPECT_NEAR(0.3, corner_keypts->points[2].y, tolerance_);
  EXPECT_NEAR(0.3, corner_keypts->points[2].z, tolerance_);
}

TEST_F(TestPointCloudUtils, ComputeKeypoints2) {
  PointCloud::Ptr corner(new PointCloud);
  PointCloud::Ptr corner_keypts(new PointCloud);
  corner = GenerateCorner();

  Normals::Ptr corner_normals(new Normals);
  ExtractNormals(corner, corner_normals);

  HarrisParams params = GenerateHarrisParams();

  ComputeKeypoints(corner, corner_normals, params, 4, corner_keypts);

  EXPECT_EQ(3, corner_keypts->size());
  EXPECT_NEAR(0, corner_keypts->points[0].x, tolerance_);
  EXPECT_NEAR(0, corner_keypts->points[0].y, tolerance_);
  EXPECT_NEAR(0, corner_keypts->points[0].z, tolerance_);
}

TEST_F(TestPointCloudUtils, ComputeFeatures) {
  PointCloud::Ptr corner(new PointCloud);
  PointCloud::Ptr corner_keypts(new PointCloud);
  corner = GenerateCorner();

  Normals::Ptr corner_normals(new Normals);
  ExtractNormals(corner, corner_normals);

  HarrisParams params = GenerateHarrisParams();

  ComputeKeypoints(corner, corner_normals, params, 4, corner_keypts);

  Features::Ptr corner_features(new Features);
  ComputeFeatures(
      corner_keypts, corner, corner_normals, 1.0, 4, corner_features);

  EXPECT_EQ(3, corner_features->size());
}

TEST_F(TestPointCloudUtils, ComputeIcpObservability) {
  Eigen::Matrix<double, 3, 1> eigenvalues_new =
      Eigen::Matrix<double, 3, 1>::Zero();
  auto query = GeneratePlane();
  ComputeIcpObservability(query, &eigenvalues_new);
  // Note Eigen sorts this automatically
  EXPECT_NEAR(eigenvalues_new(0), 0, tolerance_);
  EXPECT_NEAR(eigenvalues_new(1), 0, tolerance_);
  EXPECT_NEAR(eigenvalues_new(2), 100, tolerance_);
}

TEST_F(TestPointCloudUtils, ComputeAp_ForPoint2PlaneICP) {
  PointCloud::Ptr plane(new PointCloud);
  plane = GeneratePlane();
  Normals::Ptr plane_normals(new Normals);
  Eigen::Matrix<double, 6, 6> Ap = Eigen::Matrix<double, 6, 6>::Zero();
  PointCloud::Ptr plane_normalized(new PointCloud);
  ExtractNormals(plane, plane_normals);
  NormalizePCloud(plane, plane_normalized);
  std::vector<size_t> correspondences(plane->size());
  std::iota(std::begin(correspondences), std::end(correspondences), 0);
  // Generate Ap manually
  Eigen::MatrixXf Ap_ref = Eigen::MatrixXf::Zero(6, 6);
  Eigen::Vector3d a_i, n_i;
  for (size_t i = 0; i < plane->size(); i++) {
    a_i << plane_normalized->points[i].x, //////
        plane_normalized->points[i].y,    //////
        plane_normalized->points[i].z;

    n_i << plane_normals->points[correspondences[i]].normal_x, //////
        plane_normals->points[correspondences[i]].normal_y,    //////
        plane_normals->points[correspondences[i]].normal_z;
    double a, b, c;
    a = (a_i.cross(n_i))(0);
    b = (a_i.cross(n_i))(1);
    c = n_i(2);
    Ap_ref(0, 0) += a * a;
    Ap_ref(0, 1) += a * b;
    Ap_ref(1, 0) += a * b;
    Ap_ref(1, 1) += b * b;
    Ap_ref(0, 5) += a * c;
    Ap_ref(1, 5) += b * c;
    Ap_ref(5, 0) += a * c;
    Ap_ref(5, 1) += b * c;
    Ap_ref(5, 5) += c * c;
  }

  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  ComputeAp_ForPoint2PlaneICP(
      plane_normalized, plane_normals, correspondences, T, Ap);

  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 6; j++) {
      EXPECT_NEAR(Ap(i, j), Ap_ref(i, j), tolerance_);
    }
  }
  EXPECT_NEAR(Ap(0, 0), 56.77534, tolerance_);
  EXPECT_NEAR(Ap(1, 1), 56.77534, tolerance_);
  EXPECT_NEAR(Ap(5, 5), 100, tolerance_);
}

} // namespace utils

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_utils");
  return RUN_ALL_TESTS();
}
