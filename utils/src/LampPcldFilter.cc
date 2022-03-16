/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#include <pcl/filters/filter.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <utils/LampPcldFilter.h>

LampPcldFilter::LampPcldFilter(const LampPcldFilterParams& params)
  : params_(params), processed_first_cloud_(false) {
  grid_leaf_size_ = (params.adaptive_max_grid + params.adaptive_min_grid) / 2.0;
}

void LampPcldFilter::Filter(const PointCloud& original_cloud,
                            PointCloud::Ptr new_cloud) {
  AdaptiveGridFilter(params_.adaptive_grid_target,
                     params_.adaptive_min_grid,
                     params_.adaptive_max_grid,
                     original_cloud,
                     new_cloud);

  // Apply random downsampling to the keyed scan
  if (params_.random_filter) {
    const int n_points = static_cast<int>((1.0 - params_.decimate_percentage) *
                                          new_cloud->size());
    pcl::RandomSample<Point> random_filter;
    random_filter.setSample(n_points);
    random_filter.setInputCloud(new_cloud);
    random_filter.filter(*new_cloud);
  }
}

void LampPcldFilter::AdaptiveGridFilter(const double& target_pt_size,
                                        const double& min_leaf_size,
                                        const double& max_leaf_size,
                                        const PointCloud& original_cloud,
                                        PointCloud::Ptr new_cloud) {
  if (original_cloud.size() < target_pt_size) {
    *new_cloud = original_cloud;
    return;
  }

  *new_cloud = original_cloud;
  pcl::VoxelGrid<Point> grid;
  grid.setLeafSize(grid_leaf_size_, grid_leaf_size_, grid_leaf_size_);
  grid.setInputCloud(new_cloud);
  grid.filter(*new_cloud);

  double size_factor = static_cast<double>(new_cloud->size()) /
      static_cast<double>(target_pt_size);
  double obs_factor = 0.0;
  if (params_.observability_check) {
    Eigen::Matrix<double, 3, 1> obs_eigenv;
    utils::ComputeIcpObservability(new_cloud, &obs_eigenv);
    double observability =
        obs_eigenv.minCoeff() / static_cast<double>(new_cloud->size());
    if (!processed_first_cloud_)
      prev_observability_ = observability;

    obs_factor = (prev_observability_ - observability) / prev_observability_;
    prev_observability_ = observability;
  }

  grid_leaf_size_ =
      std::min(max_leaf_size,
               std::max(min_leaf_size,
                        grid_leaf_size_ *
                            (size_factor - abs(size_factor - 1) * obs_factor)));
}