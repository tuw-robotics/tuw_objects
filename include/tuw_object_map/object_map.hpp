#ifndef TUW_OBJECT_MAP__OBJECT_MAP_HPP_
#define TUW_OBJECT_MAP__OBJECT_MAP_HPP_

#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>
#include <tuw_geometry/figure.hpp>
#include <tuw_object_map_msgs/msg/object_map.hpp>
#include <tuw_object_map/geo_map.hpp>

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
    void init_map(const std::string &mapimage);
    void init_map(double latitude, double longitude, double altitude);
    void line(cv::Vec3d start, cv::Vec3d end, double bondary, double enflation);
    const cv::Mat &object_image();
    GeoMapMetaData &info(){
      return info_;
    }
    cv::Mat img_costmap_;
    cv::Mat img_map_;
  private:
    std::vector<Object> objects_;  /// pix/meter 
    GeoMapMetaData info_;
  };
}
#endif // TUW_OBJECT_MAP__OBJECT_MAP_HPP_
