#include "tuw_object_map/object_map.hpp"
#include <iostream>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <GeographicLib/UTMUPS.hpp>

using namespace tuw_object_map;

ObjectMap::ObjectMap()
{
}

void ObjectMap::process(const std::vector<Object> &objects)
{
  objects_.clear();
  for (const auto &obj : objects)
  {
    objects_.push_back(obj);
  }

  map_->symbol(tuw::Pose2D(0, 0, 0), 10.0, tuw::Figure::blue);
  map_header_.init(map_->height(), map_->width(), map_->Mw2m());
  img_objects_ = cv::Mat(map_->height(), map_->width(), CV_8U, cv::Scalar(0x80));

  for (auto &o : objects_)
  {
    tuw::Point2D a(o.geometry.points[0][0], o.geometry.points[0][1]);
    tuw::Point2D b(o.geometry.points[1][0], o.geometry.points[1][1]);
    map_->line(a, b, tuw::Figure::green);
    
    int thickness_bondary = o.geometry.bondary[0] * 2. * map_header_.scale_x();
    map_header_.line(img_objects_, a, b, tuw::Figure::white, thickness_bondary);
    int thickness_enflation = o.geometry.enflation[0] * 2. * map_header_.scale_x();
    map_header_.line(img_objects_, a, b, tuw::Figure::black, thickness_enflation);
    
  }

  cv::imshow(map_->title(), map_->view());
  cv::imshow("enflation", img_objects_);
  cv::waitKey(1000);
}

void ObjectMap::origin_world(cv::Vec3d lla)
{
  origin_lla_ = lla;
  bool northp;
  GeographicLib::UTMUPS::Forward(origin_lla_[0], origin_lla_[1], zone_utm_, northp, origin_utm_[0], origin_utm_[1]);
  origin_utm_[2] = lla[2];
}

cv::Vec3d &ObjectMap::convert_LLA_to_MAP(const cv::Vec3d &p_lla, cv::Vec3d &p_map)
{

  int zone;
  bool northp;
  double x, y, gamma, k;
  GeographicLib::UTMUPS::Forward(p_lla[0], p_lla[1], zone, northp, x, y, gamma, k, zone_utm_);
  p_map[0] = x - origin_utm_[0];
  p_map[1] = y - origin_utm_[1];
  p_map[2] = p_lla[3] - origin_utm_[3];
  return p_map;
}


void ObjectMap::init_map(double map_resolution, cv::Size2d map_size, cv::Vec2d map_offset)
{
  origin_utm_[0] +=  map_offset[0];
  origin_utm_[1] +=  map_offset[1];
  double sx = 1./ map_resolution;
  double sy = 1./-map_resolution;
  int width_pixel = map_size.width / map_resolution;
  int height_pixel = map_size.height / map_resolution; 
  double grid_scale_x = 10.;
  double grid_scale_y = 10.;
  double ox = height_pixel / 2.0;
  double oy = width_pixel / 2.0;
  double mx = 0;
  double my = 0;
  cv::Matx<double, 3, 3> Tw(1, 0,  mx, 0, 1,  my, 0, 0, 1);  // translation
  cv::Matx<double, 3, 3> Sc(sx, 0, 0, 0, sy, 0, 0, 0, 1);    // scaling
  cv::Matx<double, 3, 3> Tm(1, 0, ox, 0, 1, oy, 0, 0, 1);    // translation
  map_ = std::make_shared<tuw::Figure>("object_map");
  map_->init(width_pixel, height_pixel, Tm*Sc*Tw, grid_scale_x, grid_scale_y, "");
}

void ObjectMap::init_map(const std::string &mapimage)
{
  background_image_filename_ = mapimage + std::string("mapimage.jpg");
  cv::Mat background_image_ = cv::imread(background_image_filename_, cv::IMREAD_COLOR);
  std::string geo_info_filename = mapimage + std::string("mapimage.jgw");
  std::vector<double> numbers = read_geo_info_jgw(geo_info_filename);
  double sx = 1./ numbers[0];
  double sy = 1./ numbers[3];
  int width_pixel = background_image_.rows;
  int height_pixel = background_image_.cols;
  double grid_scale_x = 10.;
  double grid_scale_y = 10.;
  double ox = height_pixel / 2.0;
  double oy = width_pixel / 2.0;
  double mx = 0;
  double my = 0;
  origin_utm_[0] =  numbers[4] +ox* numbers[0];
  origin_utm_[1] =  numbers[5] +oy* numbers[3];
  cv::Matx<double, 3, 3> Tw(1, 0,  mx, 0, 1,  my, 0, 0, 1);  // translation
  cv::Matx<double, 3, 3> Sc(sx, 0, 0, 0, sy, 0, 0, 0, 1);    // scaling
  cv::Matx<double, 3, 3> Tm(1, 0, ox, 0, 1, oy, 0, 0, 1);    // translation
  map_ = std::make_shared<tuw::Figure>("object_map");
  map_->init(width_pixel, height_pixel, Tm*Sc*Tw, grid_scale_x, grid_scale_y, background_image_filename_);
}

std::vector<double> ObjectMap::read_geo_info_jgw(const std::string &filename)
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