//
// Created by linglx on 18-4-16.
//

#include "roi_filter.h"

namespace robosense
{
namespace localization
{
namespace filter
{
void ROIFilter::setBound(float left, float right, float back, float front, float down, float up)
{
  option_.left_bound = left;
  option_.right_bound = right;
  option_.back_bound = back;
  option_.front_bound = front;
  option_.down_bound = down;
  option_.up_bound = up;
}

void ROIFilter::filter(PointCloudTypePtr input, PointCloudTypePtr output, int num_of_out)
{
  option_.scale_of_filter = input->size() / num_of_out;
  initCells(point_cloud_cells_);
  initBound();
  pointCloudToGridCells(input, point_cloud_cells_);
  showPointCloud(point_cloud_cells_);
}

void ROIFilter::filter(PointCloudTypePtr input, PointCloudTypePtr output, int num_of_out, int row_cell, int column_cell)
{
  option_.scale_of_filter = num_of_out / input->size() ;
  option_.row_cell = row_cell;
  option_.column_cell = column_cell;
  initCells(point_cloud_cells_);
  initBound();
  pointCloudToGridCells(input, point_cloud_cells_);
}

void ROIFilter::initCells(vector<PointCloud<pcl::PointXYZI>::Ptr>& cells)
{
  for (int i = 0; i < option_.row_cell * option_.column_cell; ++i)
  {
    PointCloudTypePtr cell(new PointCloudType);
    cells.push_back(cell);
  }
}

void ROIFilter::initBound()
{
  row_seg_points_ = new float[option_.row_cell + 1];
  column_seg_points_ = new float[option_.column_cell + 1];
  for (int i = 0; i < option_.row_cell + 1; ++i)
  {
    row_seg_points_[i] = option_.left_bound + i * (option_.right_bound - option_.left_bound) / option_.row_cell;
  }
  for (int j = 0; j < option_.column_cell + 1; ++j)
  {
    column_seg_points_[j] = option_.back_bound + j * (option_.front_bound - option_.back_bound) / option_.column_cell;
  }
}

void ROIFilter::pointCloudToGridCells(PointCloudTypeConstPtr point_cloud, vector<PointCloudTypePtr>& cells)
{
  int which_cell = 0;
  for (unsigned long k = 0; k < point_cloud->size(); ++k)
  {
    which_cell = whichCellOfPoint(point_cloud->at(k).x, point_cloud->at(k).y);
    if (which_cell >= 0 && which_cell < option_.row_cell * option_.column_cell)
    {
      point_cloud_cells_[which_cell]->push_back(point_cloud->at(k));
    }
  }
}

int ROIFilter::whichCellOfPoint(float x, float y)
{
  int x_pos = 0;
  if (x <= option_.left_bound || x > option_.right_bound)
    x_pos = -1;
  else
    x_pos = bs(row_seg_points_, option_.row_cell + 1, x);  // 1, 2, ..., row_cell

  int y_pos = 0;
  if (y <= option_.back_bound || y > option_.front_bound)
    y_pos = -1;
  else
    y_pos = bs(column_seg_points_, option_.column_cell + 1, y);  // 1, 2, ..., column_cell

  if (x_pos != -1 && y_pos != -1)
    return (y_pos - 1) * option_.row_cell + x_pos - 1;  // 0, 1, row_cell * column_cell - 1
  else
    return -1;
}

int ROIFilter::bs(float* a, int len, float item)
{
  // int left = 0; int right = len - 1;
  unsigned long pos = lower_bound(a, a + len, item) - a;
  return int(pos);
}

void ROIFilter::filter(PointCloudTypePtr point_cloud)
{
}

void ROIFilter::showPointCloud(vector<PointCloudTypePtr> &plane_ptr_list)
{
  string name = "grid";
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(name));
  int color[3] = {0};
  viewer->addCoordinateSystem(1);
  for (int i = 0; i < plane_ptr_list.size(); ++i)
  {
    color[0] = static_cast<uint8_t>(255 * (1024 * rand() / (RAND_MAX + 1.0f)));
    color[1] = static_cast<uint8_t>(255 * (1024 * rand() / (RAND_MAX + 1.0f)));
    color[2] = static_cast<uint8_t>(255 * (1024 * rand() / (RAND_MAX + 1.0f)));
    pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(plane_ptr_list[i], color[0], color[1],
                                                                                  color[2]);
    std::string plane_name = name + "_" + boost::to_string(i);
    viewer->addPointCloud(plane_ptr_list[i], color_handler, plane_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, plane_name);
    viewer->setSize(800, 600);
  }
  while (!viewer->wasStopped())
  {
    viewer->spinOnce();
  }
}

}
}
}
