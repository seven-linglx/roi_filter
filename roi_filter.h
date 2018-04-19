//
// Created by linglx on 18-4-16.
//

#ifndef PROJECTS_ROI_FILTER_H
#define PROJECTS_ROI_FILTER_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
#include <pcl/sample_consensus/method_types.h>  //随机样本一致性算法 方法类型
#include <pcl/segmentation/sac_segmentation.h>  //随机样本一致性算法 分割方法
#include <pcl/console/time.h>

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
      threshold_of_plane_seg = 0.3;
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
    float threshold_of_plane_seg;
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
  unsigned int pointCloudToGridCells(PointCloudTypeConstPtr point_cloud, vector<PointCloudTypePtr>& cells);
  int whichCellOfPoint(float x, float y, float z);
  int bs(float a[], int len, float item); // binary search
  void filterCells(vector<PointCloudTypePtr> &cells, PointCloudTypePtr point_cloud);
  void segment(PointCloudTypePtr in);
  void ransomFilter(PointCloudTypePtr in, PointCloudTypePtr out, float scale);
  void showPointCloud(vector<PointCloudTypePtr> &cells);
};

}// namespace filter
}// namespace localization
}// namespace robosense
#endif  // PROJECTS_ROI_FILTER_H
