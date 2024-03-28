#include "tuw_geo_map/geo_map_node.hpp"
#include <tuw_json/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>

using std::placeholders::_1;
using std::placeholders::_2;

using namespace tuw_geo_map;

GeoMapNode::GeoMapNode(const std::string &node_name)
    : Node(node_name)
{
  declare_parameters();
  read_parameters();
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  using namespace std::chrono_literals;
  timer_ = create_wall_timer(100ms, std::bind(&GeoMapNode::callback_timer, this));
}

void GeoMapNode::callback_timer()
{
  // RCLCPP_INFO(this->get_logger(), "on_timer");
  publish_transforms();
}


void GeoMapNode::publish_transforms()
{
  if (publish_utm_)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = frame_utm_;
    tf.child_frame_id = frame_map_;
    tf.transform.translation.x = info_.utm()[0];
    tf.transform.translation.y = info_.utm()[1];
    tf.transform.translation.z = info_.utm()[2];
    RCLCPP_INFO_ONCE(this->get_logger(), "publish TF: frame_id: %s, child_frame_id: %s", tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
    tf_broadcaster_->sendTransform(tf);
  }
}

void GeoMapNode::declare_parameters()
{
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "map topic";
    this->declare_parameter<std::string>("map_topic", "geo_map", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "frame_map";
    this->declare_parameter<std::string>("frame_map", "geo_map", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "frame_utm only need if publish_utm == true";
    this->declare_parameter<std::string>("frame_utm", "utm", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "origin latitude";
    this->declare_parameter<double>("origin_latitude", 46.80213975, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "origin longitude";
    this->declare_parameter<double>("origin_longitude", 15.83715523, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "origin altitude";
    this->declare_parameter<double>("origin_altitude", 338.917, descriptor);
  }
}

void GeoMapNode::read_parameters()
{
  double latitude, longitude, altitude;
  this->get_parameter<std::string>("map_topic", map_topic_);
  RCLCPP_INFO(this->get_logger(), "map_topic: %s", map_topic_.c_str());
  this->get_parameter<std::string>("frame_map", frame_map_);
  this->get_parameter<std::string>("frame_utm", frame_utm_);
  this->get_parameter<double>("origin_latitude", latitude);
  this->get_parameter<double>("origin_longitude", longitude);
  this->get_parameter<double>("origin_altitude", altitude);
  this->get_parameter<bool>("publish_utm", publish_utm_);


  info_.init(latitude, longitude, altitude);
  RCLCPP_INFO(this->get_logger(), "%s", info_.info_geo().c_str());

}
