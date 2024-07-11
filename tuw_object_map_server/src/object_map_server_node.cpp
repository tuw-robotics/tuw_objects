#include "tuw_object_map_server/object_map_server_node.hpp"
#include <tuw_object_map_msgs/object_map_json.hpp>
#include <tuw_json/json.hpp>
#include <filesystem>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using namespace tuw_object_map_server;

ObjectMapServerNode::ObjectMapServerNode(const std::string &node_name)
    : Node(node_name)
{
  declare_parameters();
  read_parameters();

  read_object_map(object_map_json_);
  pub_object_map_ = this->create_publisher<tuw_object_map_msgs::msg::ObjectMap>(topic_name_objects_, 10);
  publish_map();

  if(loop_rate_ > 0){
    using namespace std::chrono_literals;
    timer_ = create_wall_timer(1000ms * loop_rate_, std::bind(&ObjectMapServerNode::callback_timer, this));
  } 
  // Create a service that provides the occupancy grid
  srv_map_ = create_service<tuw_object_map_msgs::srv::GetObjectMap>(
    service_name_objects_,
    std::bind(&ObjectMapServerNode::callback_get_map, this, _1, _2, _3));
}

void ObjectMapServerNode::read_object_map(const std::string &filename)
{
  object_map_ = std::make_shared<tuw_object_map_msgs::msg::ObjectMap>();
  tuw_json::fromJson(tuw_json::read(filename, "object_map"), *object_map_);
  if(!frame_id_.empty()){
    object_map_->header.frame_id =  frame_id_;
    object_map_->header.stamp = this->get_clock()->now();
  }
}

void ObjectMapServerNode::callback_get_map(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<tuw_object_map_msgs::srv::GetObjectMap::Request>/*request*/,
  std::shared_ptr<tuw_object_map_msgs::srv::GetObjectMap::Response> response)
{
  RCLCPP_INFO(get_logger(), "Handling GetMap request");
  response->map = *object_map_;
}

void ObjectMapServerNode::callback_timer()
{
  RCLCPP_INFO(this->get_logger(), "on_timer");
  publish_map();
}


void ObjectMapServerNode::publish_map()
{
  if(!object_map_) return;
  pub_object_map_->publish(*object_map_);
}

void ObjectMapServerNode::declare_parameters()
{
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "Name of the frame";
    this->declare_parameter<std::string>("frame_id", "WGS84", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "Filename to load the object map from a json file if not set the node will wait for a msg on the topic";
    this->declare_parameter<std::string>("object_map_json", "", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "loop or publishing rate in seconds. If 0 or less, the graph is published once";
    this->declare_parameter<int>("loop_rate", 10, descriptor);
  }
}

void ObjectMapServerNode::read_parameters()
{
  this->get_parameter<std::string>("frame_id", frame_id_);
  RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
  this->get_parameter<std::string>("object_map_json", object_map_json_);
  RCLCPP_INFO(this->get_logger(), "object_map_json: %s", object_map_json_.c_str());
  this->get_parameter<int>("loop_rate", loop_rate_);
  RCLCPP_INFO_ONCE(this->get_logger(), "loop_rate %4d", loop_rate_);

  if (!object_map_json_.empty())
  {
    if (!std::filesystem::exists(object_map_json_) || !std::filesystem::is_regular_file(object_map_json_))
      RCLCPP_ERROR(this->get_logger(), "Error: %s des not exist", object_map_json_.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Error: object_map_json not set");
  }
}
