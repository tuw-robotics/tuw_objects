#ifndef TUW_OBJECT_MAP__OBJECT_MAP_HPP_
#define TUW_OBJECT_MAP__OBJECT_MAP_HPP_

#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>
#include <tuw_geometry/geo_handler.hpp>
#include <tuw_object_map_msgs/msg/object_map.hpp>

namespace tuw_object_map
{

  class ObjectMap : public tuw::GeoHdl
  {
  public:
    enum Cell : uint8_t
    {
      CELL_FREE = 0xFF,
      CELL_UNKNOWN = 0x80,
      CELL_OCCUPIED = 0x00
    };
    ObjectMap();
    cv::Mat &mat();
    void draw(const tuw_object_map_msgs::msg::ObjectMap &msg);

    void line(cv::Vec3d start, cv::Vec3d end, Cell value, double size);
    void line(cv::Vec3d start, cv::Vec3d end, double bondary, double enflation);
    cv::Mat &img_map()
    {
      return img_map_;
    }
    cv::Mat_<uint8_t> &img_costmap()
    {
      return img_costmap_;
    }
    void imshow(int delay = 10);

  private:
    cv::Mat_<uint8_t> img_costmap_;
    cv::Mat img_map_;
  };
}
#endif // TUW_OBJECT_MAP__OBJECT_MAP_HPP_
