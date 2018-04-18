//
// Created by linglx on 18-4-16.
//

#ifndef PROJECTS_ROI_FILTER_H
#define PROJECTS_ROI_FILTER_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>  //allows us to use pcl::transformPointCloud function
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>  //随机样本一致性算法 方法类型
#include <pcl/segmentation/sac_segmentation.h>  //随机样本一致性算法 分割方法
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

namespace robosense
{
namespace localization
{
namespace filter
{
using namespace std;
using namespace pcl;

#define PointType PointXYZI
#define PointCloudType PointCloud<PointType>
#define PointCloudTypePtr PointCloud<PointType>::Ptr
#define PointCloudTypeConstPtr PointCloud<PointType>::ConstPtr

class ROIFilter
{
private:
  struct Options
  {
    Options()
    {
      left_bound = -15;
      right_bound = 15;
      back_bound = -30;
      front_bound = 30;
      down_bound = -2;
      up_bound = 5;
      row_cell = 4;
      column_cell = 4;
      scale_of_filter = 0.1;
    }
    float left_bound;
    float right_bound;
    float back_bound;
    float front_bound;
    float down_bound;
    float up_bound;
    int row_cell;
    int column_cell;
    float scale_of_filter;
  };

public:
  void filter(PointCloudTypePtr input, PointCloudTypePtr output, int num_of_out);
  void filter(PointCloudTypePtr input, PointCloudTypePtr output, int num_of_out, int row_cell, int column_cell);
  void setBound(float left, float right, float back, float front, float down, float up);

private:
  Options option_;
  vector<PointCloudTypePtr> point_cloud_cells_;
  float* row_seg_points_;
  float* column_seg_points_;
  void initCells(vector<PointCloudTypePtr>& cells);
  void initBound();
  void pointCloudToGridCells(PointCloudTypeConstPtr point_cloud, vector<PointCloudTypePtr>& cells);
  int whichCellOfPoint(float x, float y);
  int bs(float a[], int len, float item); // binary search
  void filter(PointCloudTypePtr point_cloud);
  void segment(PointCloudTypePtr point_cloud);
  void showPointCloud(vector<PointCloudTypePtr> &plane_ptr_list);
};

}  // namespace filter
}  // namespace localization
}  // namespace robosense
#endif  // PROJECTS_ROI_FILTER_H
