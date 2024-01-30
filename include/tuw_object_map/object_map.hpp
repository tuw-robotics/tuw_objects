#ifndef TUW_OBJECT_MAP__OBJECT_MAP_HPP_
#define TUW_OBJECT_MAP__OBJECT_MAP_HPP_

#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>
#include <tuw_object_map_msgs/msg/object_map.hpp>

namespace tuw_object_map
{

  class Objects
  {
  public:
    uint64_t id;
    uint32_t type;
    std::vector<cv::Vec3d> lla;
    std::vector<cv::Vec3d> utm;
    std::vector<cv::Point2d> pi;
    std::vector<double> enflation;
    std::vector<double> bondary;
  };

  class ObjectMap
  {
  public:
    ObjectMap();

    void process(const Objects &objects);
  };
}
#endif // TUW_OBJECT_MAP__OBJECT_MAP_HPP_
