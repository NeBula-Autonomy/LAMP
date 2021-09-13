/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#include <utils/CommonFunctions.h>
#include <utils/CommonStructs.h>
#include <utils/PointCloudUtils.h>

struct LampPcldFilterParams {
  bool random_filter;
  // Percentage of points to discard. Must be between 0.0 and 1.0;
  double decimate_percentage;
  // Adaptive filter
  bool adaptive_grid_filter;
  // Adaptive point size for adaptive grid filter
  int adaptive_grid_target;
  // Adaptive constraint
  double adaptive_max_grid;
  double adaptive_min_grid;
  bool observability_check;
};

class LampPcldFilter {
public:
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