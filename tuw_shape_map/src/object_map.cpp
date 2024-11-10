#include "tuw_shape_map/object_map.hpp"
#include "tuw_object_msgs/shape.hpp"
#include "tuw_object_msgs/shape_array.hpp"
#include "tuw_std_msgs/parameter_array.hpp"

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
    tuw::Point2D p_world = static_cast<tuw::GeoHdl *>(userdata)->m2w(tuw::Point2D(x, y));
    cv::Vec3d p_utm = static_cast<tuw::GeoHdl *>(userdata)->world2utm(cv::Vec3d(p_world.x(), p_world.y(), 0));
    cv::Vec3d p_lla = static_cast<tuw::GeoHdl *>(userdata)->utm2lla(p_utm);
    printf("map: [%5d, %5d] px --> world: [%6.3fm, %6.3fm] --> {\"latitude\":%10.8f,\"longitude\":%10.8f,\"altitude\":%6.3f}\n",
           x, y, p_world.x(), p_world.y(), p_lla[0], p_lla[1], p_lla[2]);
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
  cv::Point a = lla2map(start);
  cv::Point b = lla2map(end);
  if (size > 0)
  {
    int thickness = size * 2. / this->resolution_x();
    cv::line(img_costmap_, a, b, cv::Scalar(value, value, value), thickness);
  }
  if (!img_map_.empty())
    cv::line(img_map_, a, b, cv::Scalar(0, 0xFF, 0), 1);
}

void ObjectMap::line(cv::Vec3d start, cv::Vec3d end, double bondary, double enflation)
{
  cv::Point a = lla2map(start);
  cv::Point b = lla2map(end);
  if (bondary > 0)
  {
    int thickness_bondary = bondary * 2. / this->resolution_x();
    cv::line(img_costmap_, a, b, cv::Scalar(Cell::CELL_FREE, Cell::CELL_FREE, Cell::CELL_FREE), thickness_bondary);
  }
  if (enflation > 0)
  {
    int thickness_enflation = enflation * 2. / this->resolution_x();
    cv::line(img_costmap_, a, b, cv::Scalar(Cell::CELL_OCCUPIED, Cell::CELL_OCCUPIED, Cell::CELL_OCCUPIED), thickness_enflation);
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

cv::Mat &ObjectMap::mat()
{
  return img_costmap_;
}

void ObjectMap::comptue_map_points(tuw_object_msgs::msg::ShapeArray &msg) const
{
  
  /*
  for (auto &o : msg.shapes)
  {
    o.points.clear();
    cv::Vec3d p_world, p_lla, p_utm;
    for (const auto &geo_point : o.points)
    {
      p_lla = cv::Vec3d(geo_point.latitude, geo_point.longitude, geo_point.altitude);
      lla2utm(p_lla, p_utm);
      utm2world(p_utm, p_world);
      geometry_msgs::msg::Point p;
      p.x = p_world[0], p.y = p_world[1], p.z = p_world[2];
      o.map_points.push_back(p);
    }
  }
  */
}

void ObjectMap::draw(const tuw_object_msgs::msg::ShapeArray &msg)
{

  /// Frist draw free space
  for (const auto &o : msg.shapes)
  {
    cv::Vec3d p0, p1;
    if (o.type == tuw_object_msgs::msg::Shape::TYPE_PLANT_WINE_ROW)
    {
      if (o.points.size() > 0)
      {
        double free = static_cast<const tuw_std_msgs::ParameterArray&>(o.params_points[0]).value<double>("free");
        p0 = cv::Vec3d(o.points[0].x, o.points[0].y, o.points[0].z);
        line(p0, p0, ObjectMap::CELL_FREE, free);
      }
      for (size_t i = 1; i < o.points.size(); i++)
      {
        double free = static_cast<const tuw_std_msgs::ParameterArray&>(o.params_points[i]).value<double>("free");
        p1 = cv::Vec3d(o.points[i].x, o.points[i].y, o.points[i].z);
        line(p0, p1, ObjectMap::CELL_FREE, free);
        p0 = p1;
      }
    }
    else if ((o.type == tuw_object_msgs::msg::Shape::TYPE_TRANSIT) ||
             (o.type == tuw_object_msgs::msg::Shape::TYPE_TRANSIT_STREET) ||
             (o.type == tuw_object_msgs::msg::Shape::TYPE_TRANSIT_GRAVEL))
    {
      if (o.points.size() > 0)
      {
        double free = static_cast<const tuw_std_msgs::ParameterArray&>(o.params_points[0]).value<double>("free");
        p0 = cv::Vec3d(o.points[0].x, o.points[0].y, o.points[0].z);
        line(p0, p0, ObjectMap::CELL_FREE, free);
      }
      for (size_t i = 1; i < o.points.size(); i++)
      {
        double free = static_cast<const tuw_std_msgs::ParameterArray&>(o.params_points[i]).value<double>("free");
        p1 = cv::Vec3d(o.points[i].x, o.points[i].y, o.points[i].z);
        line(p0, p1, ObjectMap::CELL_FREE, free);
        p0 = p1;
      }
    }
  }
  /// Frist draw occupied space
  for (const auto &o : msg.shapes)
  {
    cv::Vec3d p0, p1;
    if (o.type == tuw_object_msgs::msg::Shape::TYPE_PLANT_WINE_ROW)
    {
      if (o.points.size() > 0)
      {
        double occupied = static_cast<const tuw_std_msgs::ParameterArray&>(o.params_points[0]).value<double>("occupied");
        p0 = cv::Vec3d(o.points[0].x, o.points[0].y, o.points[0].z);
        line(p0, p0, ObjectMap::CELL_OCCUPIED, occupied);
      }
      for (size_t i = 1; i < o.points.size(); i++)
      {
        double occupied = static_cast<const tuw_std_msgs::ParameterArray&>(o.params_points[i]).value<double>("occupied");
        p1 = cv::Vec3d(o.points[i].x, o.points[i].y, o.points[i].z);
        line(p0, p1, ObjectMap::CELL_OCCUPIED, occupied);
        p0 = p1;
      }
    }
    else if ((o.type == tuw_object_msgs::msg::Shape::TYPE_OBSTACLE_TREE))
    {
      cv::Vec3d p0, p1;
      if (o.points.size() > 0)
      {
        double occupied = static_cast<const tuw_std_msgs::ParameterArray&>(o.params_points[0]).value<double>("occupied");
        p0 = cv::Vec3d(o.points[0].x, o.points[0].y, o.points[0].z);
        line(p0, p0, ObjectMap::CELL_OCCUPIED, occupied);
      }
      for (size_t i = 1; i < o.points.size(); i++)
      {
        double occupied = static_cast<const tuw_std_msgs::ParameterArray&>(o.params_points[i]).value<double>("occupied");
        p1 = cv::Vec3d(o.points[i].y, o.points[i].x, o.points[i].z);
        line(p0, p1, ObjectMap::CELL_OCCUPIED, occupied);
        p0 = p1;
      }
    }
  }
  /// fill corners (for stage map)
  img_costmap()(0, 0) = ObjectMap::CELL_OCCUPIED;
  img_costmap()(img_costmap().rows - 1, 0) = ObjectMap::CELL_OCCUPIED;
  img_costmap()(img_costmap().rows - 1, img_costmap().cols - 1) = ObjectMap::CELL_OCCUPIED;
  img_costmap()(0, img_costmap().cols - 1) = ObjectMap::CELL_OCCUPIED;
}
