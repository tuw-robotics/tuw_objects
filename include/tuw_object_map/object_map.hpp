#ifndef TUW_OBJECT_MAP__OBJECT_MAP_HPP_
#define TUW_OBJECT_MAP__OBJECT_MAP_HPP_

#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>
#include <tuw_object_map_msgs/msg/object_map.hpp>

namespace tuw_object_map
{

  class Geometry
  {
  public:
    enum GeometryType : uint32_t{
      Point = 100,
      LineString = 200,
      Polygon = 300,
    };
    GeometryType type;
    std::vector<cv::Vec3d> points;
    std::vector<double> enflation;
    std::vector<double> bondary;
  };

  class Object
  {
  public:
    uint64_t id;
    uint32_t type;
    Geometry geometry;
  };

  class ObjectMap
  {
  public:
    ObjectMap();


    void process(const std::vector<Object> &objects);
    void set_origin(cv::Vec3d lla, double resolution);
  private:
    cv::Vec3d origin_lla_;
    cv::Vec3d origin_utm_;
    int zone_utm_;
    double map_resolution_;
    std::vector<Object> objects_utm_;
    std::vector<Object> objects_;

  public:
    cv::Vec3d &convert_LLA_to_MAP(const cv::Vec3d &lla, cv::Vec3d &map);
  };
}
#endif // TUW_OBJECT_MAP__OBJECT_MAP_HPP_
