#ifndef TUW_OBJECT_MAP__OBJECT_MAP_NODE_HPP_
#define TUW_OBJECT_MAP__OBJECT_MAP_NODE_HPP_

#include <memory>
#include <opencv2/core/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <tuw_object_map/object_map.hpp>
#include <tuw_object_map_msgs/msg/object_map.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tuw_object_map_msgs/srv/load_map.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace tuw_object_map
{
  class ObjectMapNode : public rclcpp::Node
  {
  public:
    ObjectMapNode(const std::string &node_name);

  private:
    void callback_object_map(const tuw_object_map_msgs::msg::ObjectMap::SharedPtr msg);
    void callback_load_map(const std::shared_ptr<tuw_object_map_msgs::srv::LoadMap::Request> request, std::shared_ptr<tuw_object_map_msgs::srv::LoadMap::Response> response);
    void callback_point_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    rclcpp::Subscription<tuw_object_map_msgs::msg::ObjectMap>::ConstSharedPtr sub_map_;

    rclcpp::Service<tuw_object_map_msgs::srv::LoadMap>::SharedPtr load_map_service_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_grid_map_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_map_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_utm_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::ConstSharedPtr sub_gps_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    std::shared_ptr<visualization_msgs::msg::Marker> marker_msg_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::shared_ptr<std::thread> process_;

    rclcpp::TimerBase::SharedPtr timer_;
    // callbacks
    void callback_timer();

    void load_map(const std::string &filename);

    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map_;
    tuw_object_map_msgs::msg::ObjectMap object_map_msg_;
    std::shared_ptr<ObjectMap> object_map_;
    bool publish_tf_;
    double map_border_;
    double resolution_;
    std::string frame_map_;
    std::string frame_utm_;
    std::string json_file_;
    std::string debug_folder_;

    bool show_map_;
    void declare_parameters();
    void read_parameters();
    void publish_transforms();
    void publish_marker();
  };
}
#endif // TUW_OBJECT_MAP__OBJECT_MAP_NODE_HPP_
