/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */
#ifndef LAMP_PCLD_FILTER_H
#define LAMP_PCLD_FILTER_H

#include <utils/CommonFunctions.h>
#include <utils/CommonStructs.h>
#include <utils/PointCloudUtils.h>

struct LampPcldFilterParams {
  bool random_filter = true;
  // Percentage of points to discard. Must be between 0.0 and 1.0;
  double decimate_percentage = 0.9;
  // Adaptive filter
  bool adaptive_grid_filter = false;
  // Adaptive point size for adaptive grid filter
  int adaptive_grid_target = 1000;
  // Adaptive constraint
  double adaptive_max_grid = 1.0;
  double adaptive_min_grid = 0.1;
  bool observability_check = false;
};

class LampPcldFilter {
public:
  LampPcldFilter(){};
  LampPcldFilter(const LampPcldFilterParams& params);
  ~LampPcldFilter() = default;

  void Filter(const PointCloud& original_cloud, PointCloud::Ptr new_cloud);

private:
  void AdaptiveGridFilter(const double& target_pt_size,
                          const double& min_leaf_size,
                          const double& max_leaf_size,
                          const PointCloud& original_cloud,
                          PointCloud::Ptr new_cloud);

  LampPcldFilterParams params_;
  double grid_leaf_size_;
  double prev_observability_;
  bool processed_first_cloud_;
};

#endif