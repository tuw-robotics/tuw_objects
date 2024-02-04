#ifndef TUW_OBJECT_MAP__OBJECT_MAP_NODE_HPP_
#define TUW_OBJECT_MAP__OBJECT_MAP_NODE_HPP_

#include <memory>
#include <opencv2/core/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <tuw_object_map/object_map.hpp>
#include <tuw_object_map_msgs/msg/object_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace tuw_object_map
{
  class ObjectMapNode : public rclcpp::Node
  {
  public:
    ObjectMapNode(const std::string &node_name);

  private:
    void callback_object_map(const tuw_object_map_msgs::msg::ObjectMap::SharedPtr msg);
    rclcpp::Subscription<tuw_object_map_msgs::msg::ObjectMap>::ConstSharedPtr sub_map_;
  
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_object_costmap_;

    std::shared_ptr<std::thread> process_;

    rclcpp::TimerBase::SharedPtr timer_;
    // callbacks
    void callback_timer();

    ObjectMap object_map_;
    std::string map_topic_{"object_map"};
    std::string mapimage_folder_;
    std::string json_file_;
    cv::Mat map_;
    cv::Vec3d origin_lla_;
    double map_resolution_;
    cv::Vec3d map_origin_;
    cv::Size map_size_;
    GeoMapMetaData info_;

    void declare_parameters();
    void read_parameters();
  };
}
#endif // TUW_OBJECT_MAP__OBJECT_MAP_NODE_HPP_
