#ifndef TUW_OBJECT_MAP__OBJECT_MAP_HPP_
#define TUW_OBJECT_MAP__OBJECT_MAP_HPP_

#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>
#include <tuw_object_map/geo_map.hpp>

namespace tuw_object_map
{

  class ObjectMap
  {
  public:
    ObjectMap();
    cv::Mat &process();
    void init_map(const std::string &mapimage);
    void init_map(double latitude, double longitude, double altitude);
    void line(cv::Vec3d start, cv::Vec3d end, double bondary, double enflation);
    const cv::Mat &object_image();
    GeoMapMetaData &info(){
      return info_;
    }
    void imshow();
  private:
    GeoMapMetaData info_;
    cv::Mat_<int8_t> img_costmap_;
    cv::Mat img_map_;
  };
}
#endif // TUW_OBJECT_MAP__OBJECT_MAP_HPP_
