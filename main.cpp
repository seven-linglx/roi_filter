//
// Created by linglx on 18-4-16.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
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
  roi_filter.setBound(-40, 40, -15, 15, -5, 15);
  roi_filter.filter(input_cloud, input_cloud, 2000);
}

