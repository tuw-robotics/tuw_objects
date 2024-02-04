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
    void origin_world(cv::Vec3d lla);
    void init_map(double map_resolution, cv::Size2d map_size, cv::Vec2d map_offset);
    void init_map(const std::string &mapimage);
    const cv::Mat &object_image();
    const GeoMapMetaData &map_meta_data();
    tuw::WorldScopedMaps map_header_;
    cv::Mat img_objects_;
  private:
    cv::Vec3d origin_lla_;
    cv::Vec3d origin_utm_;
    int zone_utm_;
    std::vector<Object> objects_;  /// pix/meter 
    tuw::FigurePtr map_;
    std::string background_image_filename_;

    static std::vector<double> read_geo_info_jgw(const std::string &filename);
  public:
    cv::Vec3d &convert_LLA_to_MAP(const cv::Vec3d &lla, cv::Vec3d &map);
  };
}
#endif // TUW_OBJECT_MAP__OBJECT_MAP_HPP_
