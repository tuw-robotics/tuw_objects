#include "tuw_object_map/object_map.hpp"
#include <iostream>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <GeographicLib/UTMUPS.hpp>

using namespace tuw_object_map;

void onMouse(int event, int x, int y, int, void *userdata)
{
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    cv::Vec3d lla = static_cast<tuw_object_map::GeoMapMetaData *>(userdata)->m2g(cv::Point(x, y));
    cv::Vec2d world = static_cast<tuw_object_map::GeoMapMetaData *>(userdata)->map2world(cv::Vec2d(x, y));
    printf("map: [%5d, %5d] px --> world: [%6.3fm, %6.3fm] --> {\"latitude\":%10.8f,\"longitude\":%10.8f,\"altitude\":%6.3f}\n", x, y, world[0], world[1], lla[0], lla[1], lla[2]);
  }
}
std::vector<double> read_geo_info_jgw(const std::string &filename)
{
  std::vector<double> numbers;
  std::ifstream geo_info_file(filename.c_str());
  // Check if the file is open
  if (!geo_info_file.is_open())
  {
    std::cerr << "Error opening the file!" << std::endl;
    return numbers; // Return an error code
  }

  // Read numbers from the file and store them in a vector
  double num;
  while (geo_info_file >> num)
  {
    numbers.push_back(num);
  }
  geo_info_file.close();
  return numbers;
}

ObjectMap::ObjectMap()
{
}

void ObjectMap::line(cv::Vec3d start, cv::Vec3d end, Cell value, double size)
{
  cv::Point a = info_.g2m(start);
  cv::Point b = info_.g2m(end);
  if (size > 0)
  {
    int thickness = size * 2. / info_.resolution;
    cv::line(img_costmap_, a, b, cv::Scalar(value), thickness);
  }
  if (!img_map_.empty())
    cv::line(img_map_, a, b, cv::Scalar(0, 0xFF, 0), 1);
}

void ObjectMap::line(cv::Vec3d start, cv::Vec3d end, double bondary, double enflation)
{
  cv::Point a = info_.g2m(start);
  cv::Point b = info_.g2m(end);
  if (bondary > 0)
  {
    int thickness_bondary = bondary * 2. / info_.resolution;
    cv::line(img_costmap_, a, b, cv::Scalar(Cell::CELL_FREE), thickness_bondary);
  }
  if (enflation > 0)
  {
    int thickness_enflation = enflation * 2. / info_.resolution;
    cv::line(img_costmap_, a, b, cv::Scalar(Cell::CELL_OCCUPIED), thickness_enflation);
  }
  if (!img_map_.empty())
    cv::line(img_map_, a, b, cv::Scalar(0, 0xFF, 0), 1);
}

void ObjectMap::imshow(int delay)
{
  cv::imshow("costmap", img_costmap_);
  cv::setMouseCallback("costmap", onMouse, &info_);
  if (!img_map_.empty())
  {
    cv::imshow("img_map", img_map_);
    cv::setMouseCallback("img_map", onMouse, &info_);
  }
  cv::waitKey(delay);
}

cv::Mat &ObjectMap::process()
{
  return img_costmap_;
}

void ObjectMap::init_map(double latitude, double longitude, double altitude)
{
  info_.init(latitude, longitude, altitude);
  img_costmap_ = cv::Mat(info_.size.height, info_.size.width, CV_8U, cv::Scalar(CELL_UNKNOWN));
}

void ObjectMap::init_map(const std::string &mapimage)
{
  std::string mapimage_filename = mapimage + std::string("mapimage.jpg");
  img_map_ = cv::imread(mapimage_filename, cv::IMREAD_COLOR);
  std::string geo_info_filename = mapimage + std::string("mapimage.jgw");
  std::vector<double> numbers = read_geo_info_jgw(geo_info_filename);
  info_.size = img_map_.size();
  info_.resolution = numbers[0];
  int zone = 33;
  bool northp = true;
  info_.init(numbers[4], numbers[5], 0.0, zone, northp);
  img_costmap_ = cv::Mat(info_.size.height, info_.size.width, CV_8U, cv::Scalar(CELL_UNKNOWN));
}
