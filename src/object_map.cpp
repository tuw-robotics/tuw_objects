#include "tuw_object_map/object_map.hpp"
#include <iostream>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <GeographicLib/UTMUPS.hpp>

using namespace tuw_object_map;

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


void ObjectMap::line(cv::Vec3d start, cv::Vec3d end, double bondary, double enflation){
    cv::Point a = info_.g2m(start);
    cv::Point b = info_.g2m(end);
    int thickness_bondary = bondary * 2. / info_.resolution;
    cv::line(img_costmap_, a, b, cv::Scalar(0xFF), thickness_bondary);
    int thickness_enflation = enflation * 2. / info_.resolution;
    cv::line(img_costmap_, a, b, cv::Scalar(0x00), thickness_enflation);
    if (!img_map_.empty())
      cv::line(img_map_, a, b, cv::Scalar(0, 0xFF, 0), 1);

}
void ObjectMap::process(const std::vector<Object> &objects)
{
  objects_.clear();
  for (const auto &obj : objects)
  {
    objects_.push_back(obj);
  }

  for (auto &o : objects_)
  {
    cv::Point a = info_.g2m(o.geometry.points[0]);
    cv::Point b = info_.g2m(o.geometry.points[1]);

    int thickness_bondary = o.geometry.bondary[0] * 2. / info_.resolution;
    cv::line(img_costmap_, a, b, cv::Scalar(0xFF), thickness_bondary);
    int thickness_enflation = o.geometry.enflation[0] * 2. / info_.resolution;
    cv::line(img_costmap_, a, b, cv::Scalar(0x00), thickness_enflation);
  }

  cv::imshow("costmap", img_costmap_);
  if (!img_map_.empty())
  {
    cv::imshow("img_map", img_map_);
  }
  cv::waitKey(1000);
}

void ObjectMap::init_map(double latitude, double longitude, double altitude){
  info_.init(latitude, longitude, altitude, true);
  img_costmap_ = cv::Mat(info_.size.height, info_.size.width, CV_8U, cv::Scalar(0x80));
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
  info_.init(numbers[4], numbers[5], 0.0, zone, northp, false);
  img_costmap_ = cv::Mat(info_.size.height, info_.size.width, CV_8U, cv::Scalar(0x80));
}
