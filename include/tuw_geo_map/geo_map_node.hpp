#ifndef TUW_GEO_MAP__GEO_MAP_NODE_HPP_
#define TUW_GEO_MAP__GEO_MAP_NODE_HPP_

#include <memory>
#include <opencv2/core/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <tuw_geometry/geo_map.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geographic_msgs/msg/geo_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace tuw_geo_map
{
  class GeoMapNode : public rclcpp::Node
  {
  public:
    GeoMapNode(const std::string &node_name);

  private:
    rclcpp::TimerBase::SharedPtr timer_loop_;
    rclcpp::TimerBase::SharedPtr timer_parameters_;

    rclcpp::Subscription<geographic_msgs::msg::GeoPose>::ConstSharedPtr sub_geo_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_map_pose_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_img_;

    // callbacks
    void callback_timer();
    void callback_geo_point();

    tuw::GeoMapMetaData info_;
    std::string map_topic_;
    std::string frame_map_;
    std::string frame_utm_;
    std::string mapimage_folder_;
    bool publish_utm_;
    void declare_parameters();
    void read_parameters();

    void publish_transforms();
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  };
}
#endif // TUW_GEO_MAP__GEO_MAP_NODE_HPP_
