//
// Created by linglx on 18-4-16.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h> //allows us to use pcl::transformPointCloud function
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h> //随机样本一致性算法 方法类型
#include <pcl/segmentation/sac_segmentation.h> //随机样本一致性算法 分割方法
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/time.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "roi_filter.h"

DEFINE_string(pcd_path, "/home/linglx/Data/pcd2vtk/local_laser_pcd/57laser.pcd", "path of pcd dir! ");

int main(int argc, char *argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(FLAGS_pcd_path, *input_cloud) == -1)
  {
    std::cerr << ("Couldn't read file source \n");
    return 0;
  }

  using namespace robosense::localization::filter;
  ROIFilter roi_filter;
  pcl::console::TicToc t;
  t.tic();
  roi_filter.setBound(-20, 20, -40, 40, -5, 15);
  roi_filter.filter(input_cloud, input_cloud, 2000);
  t.toc_print();
}

