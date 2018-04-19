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
  //cout << input->size() << endl;
  initCells(point_cloud_cells_);
  initBound();
  unsigned int sum_of_points = pointCloudToGridCells(input, point_cloud_cells_);
  option_.scale_of_filter = static_cast<float>(num_of_out) / static_cast<float>(sum_of_points);
  //cout << sum_of_points << '\t' << option_.scale_of_filter << endl;
  filterCells(point_cloud_cells_, output);
  //cout << output->size() << endl;
  showPointCloud(point_cloud_cells_);
}

void ROIFilter::filter(PointCloudTypePtr input, PointCloudTypePtr output, int num_of_out, int row_cell, int column_cell)
{
  option_.row_cell = row_cell;
  option_.column_cell = column_cell;
  filter(input, output, num_of_out);
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

//----------------------split to cell------------------------------------
unsigned int ROIFilter::pointCloudToGridCells(PointCloudTypeConstPtr point_cloud, vector<PointCloudTypePtr>& cells)
{
  int which_cell = 0;
  for (unsigned long k = 0; k < point_cloud->size(); ++k)
  {
    which_cell = whichCellOfPoint(point_cloud->at(k).x, point_cloud->at(k).y, point_cloud->at(k).z);
    if (which_cell >= 0 && which_cell < point_cloud_cells_.size())
    {
      point_cloud_cells_[which_cell]->push_back(point_cloud->at(k));
    }
  }

  unsigned int sum = 0;
  for (int i = 0; i < point_cloud_cells_.size(); ++i)
  {
    sum += point_cloud_cells_[i]->size();
  }

  return sum;
}

int ROIFilter::whichCellOfPoint(float x, float y, float z)
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

  int z_pos = 0;
  if (z < option_.down_bound || z > option_.up_bound)
    z_pos = -1;
  else
    z_pos = 1;

  if (x_pos != -1 && y_pos != -1 && z_pos != -1)
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
//----------------------end split to cell------------------------------------

void ROIFilter::filterCells(vector<PointCloudTypePtr>& cells, PointCloudTypePtr point_cloud)
{
  if (!point_cloud->empty()) point_cloud->clear();
  for (int i = 0; i < cells.size(); ++i)
  {
    segment(cells[i]);
    *point_cloud += *cells[i];
  }
}

void ROIFilter::segment(PointCloudTypePtr in)
{
  SACSegmentation<PointType> seg;
  ModelCoefficients::Ptr coefficients(new ModelCoefficients);  //存储输出的模型的系数
  PointIndices::Ptr inliers(new PointIndices);                 //存储内点，使用的点

  seg.setOptimizeCoefficients(true);  //可选设置
  seg.setModelType(SACMODEL_PLANE);   //设置模型类型，检测平面
  seg.setMethodType(SAC_RANSAC);      //设置方法【聚类或随机样本一致性】
  seg.setMaxIterations(15);
  seg.setDistanceThreshold(option_.threshold_of_plane_seg);

  PointCloudTypePtr cloud_f(new PointCloudType);
  PointCloudTypePtr out(new PointCloudType);
  PointCloudTypePtr cloud_plane(new PointCloudType);
  unsigned long nr_points = in->size();
  while (in->size() > 500 && in->size() > 0.5 * nr_points)
  {
    seg.setInputCloud(in);
    seg.segment(*inliers, *coefficients);
    if (!inliers->indices.empty())
    {
      ExtractIndices<PointType> extract;
      extract.setInputCloud(in);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*cloud_plane);

      cloud_f->clear();
      ransomFilter(cloud_plane, cloud_f, option_.scale_of_filter * 2);
      *out += *cloud_f;

      //cloud_f->clear();
      extract.setNegative(true);
      extract.filter(*in);
      //*in = *cloud_f;
    }
  }
  cloud_f->clear();
  ransomFilter(in, cloud_f, option_.scale_of_filter / 4);
  *out += *cloud_f;

  *in = *out;
}

void ROIFilter::ransomFilter(PointCloudTypePtr in, PointCloudTypePtr out, float scale)
{
  unsigned int sample = static_cast<unsigned int>(scale * in->size());
  unsigned long max_index = in->size();
  unsigned long choose_index = 0;
  srand((unsigned)time(NULL));
  for (int i = 0; i < sample; ++i)
  {
    choose_index = rand() % max_index;
    out->push_back(in->at(choose_index));
  }
}

void ROIFilter::showPointCloud(vector<PointCloudTypePtr>& cells)
{
  string name = "grid";
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(name));
  int color[3] = { 0 };
  viewer->addCoordinateSystem(1);
  for (int i = 0; i < cells.size(); ++i)
  {
    color[0] = static_cast<uint8_t>(255 * (1024 * rand() / (RAND_MAX + 1.0f)));
    color[1] = static_cast<uint8_t>(255 * (1024 * rand() / (RAND_MAX + 1.0f)));
    color[2] = static_cast<uint8_t>(255 * (1024 * rand() / (RAND_MAX + 1.0f)));
    pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(cells[i], color[0], color[1], color[2]);
    std::string plane_name = name + "_" + boost::to_string(i);
    viewer->addPointCloud(cells[i], color_handler, plane_name);
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
