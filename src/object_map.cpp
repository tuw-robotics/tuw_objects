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
    cv::Vec3d lla = static_cast<tuw::GeoMapMetaData *>(userdata)->m2g(cv::Point(x, y));
    cv::Vec2d world = static_cast<tuw::GeoMapMetaData *>(userdata)->map2world(cv::Vec2d(x, y));
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
  cv::Point a = g2m(start);
  cv::Point b = g2m(end);
  if (size > 0)
  {
    int thickness = size * 2. / resolution;
    cv::line(img_costmap_, a, b, cv::Scalar(value), thickness);
  }
  if (!img_map_.empty())
    cv::line(img_map_, a, b, cv::Scalar(0, 0xFF, 0), 1);
}

void ObjectMap::line(cv::Vec3d start, cv::Vec3d end, double bondary, double enflation)
{
  cv::Point a = g2m(start);
  cv::Point b = g2m(end);
  if (bondary > 0)
  {
    int thickness_bondary = bondary * 2. / resolution;
    cv::line(img_costmap_, a, b, cv::Scalar(Cell::CELL_FREE), thickness_bondary);
  }
  if (enflation > 0)
  {
    int thickness_enflation = enflation * 2. / resolution;
    cv::line(img_costmap_, a, b, cv::Scalar(Cell::CELL_OCCUPIED), thickness_enflation);
  }
  if (!img_map_.empty())
    cv::line(img_map_, a, b, cv::Scalar(0, 0xFF, 0), 1);
}

void ObjectMap::imshow(int delay)
{
  cv::imshow("costmap", img_costmap_);
  cv::setMouseCallback("costmap", onMouse, this);
  if (!img_map_.empty())
  {
    cv::imshow("img_map", img_map_);
    cv::setMouseCallback("img_map", onMouse, this);
  }
  cv::waitKey(delay);
}

cv::Mat &ObjectMap::process()
{
  return img_costmap_;
}

void ObjectMap::init(double latitude, double longitude, double altitude)
{
  tuw::GeoMapMetaData::init(latitude, longitude, altitude);
  img_costmap_ = cv::Mat(size.height, size.width, CV_8U, cv::Scalar(CELL_UNKNOWN));
}
