
#include <filesystem>
#include <json/json.h>
#include <tuw_json/json.hpp>
#include <tuw_shape_array/shape_server_node.hpp>
#include <tuw_object_msgs/shape_array_json.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using namespace tuw_objects;

ShapeServerNode::ShapeServerNode(const std::string &node_name)
    : Node(node_name)
{
  declare_parameters();
  read_parameters();

  read_shape_array(shapes_json_);
  pub_shapes_ = this->create_publisher<tuw_object_msgs::msg::ShapeArray>(topic_name_, 10);
  publish_shapes();

  if(pub_interval_ > 0){
    using namespace std::chrono_literals;
    timer_ = create_wall_timer(1000ms * pub_interval_, std::bind(&ShapeServerNode::callback_timer, this));
  } else {
    RCLCPP_INFO(get_logger(), "The objects are only published once becaue of pub_interval:=0. The node is waiting for service requests");
  }
  // Create a service that provides the occupancy grid
  srv_shapes_ = create_service<tuw_object_msgs::srv::GetShapeArray>(
    service_get_,
    std::bind(&ShapeServerNode::callback_get_shapes, this, _1, _2, _3));

  srv_publish_ = create_service<std_srvs::srv::Trigger>(
    service_publish_,
    std::bind(&ShapeServerNode::callback_publish, this, _1, _2, _3));
}

void ShapeServerNode::read_shape_array(const std::string &filename)
{
  shapes_ = std::make_shared<tuw_object_msgs::msg::ShapeArray>();
  tuw_json::fromJson(tuw_json::read(filename, "shapes"), *shapes_);
  if(!frame_id_.empty()){
    shapes_->header.frame_id =  frame_id_;
    shapes_->header.stamp = this->get_clock()->now();
  }
  if(shapes_->shapes.empty()){
    RCLCPP_ERROR(this->get_logger(), "No objects in json. Check tag name!");
    return;
  }
}

void ShapeServerNode::callback_get_shapes(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<tuw_object_msgs::srv::GetShapeArray::Request>/*request*/,
  std::shared_ptr<tuw_object_msgs::srv::GetShapeArray::Response> response)
{
  RCLCPP_INFO(get_logger(), "Handling GetMap request");
  response->shapes = *shapes_;
}

void ShapeServerNode::callback_publish(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "Handling publish request");
  if(shapes_){
    response->success = true;
    publish_shapes();
  }
}

void ShapeServerNode::callback_timer()
{
  publish_shapes();
}


void ShapeServerNode::publish_shapes()
{
  if(!shapes_) return;
  RCLCPP_INFO(this->get_logger(), "publish_objects");
  pub_shapes_->publish(*shapes_);
}

void ShapeServerNode::declare_parameters()
{
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "Name of the frame";
    this->declare_parameter<std::string>("frame_id", "WGS84", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "Filename to load shapes from a json file if not set the node will wait for a msg on the topic";
    this->declare_parameter<std::string>("json", "", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "publishing interval in seconds. If 0 or less, the graph is published once.";
    this->declare_parameter<int>("pub_interval", 10, descriptor);
  }
}

void ShapeServerNode::read_parameters()
{
  this->get_parameter<std::string>("frame_id", frame_id_);
  RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
  this->get_parameter<std::string>("json", shapes_json_);
  RCLCPP_INFO(this->get_logger(), "json: %s", shapes_json_.c_str());
  this->get_parameter<int>("pub_interval", pub_interval_);
  RCLCPP_INFO_ONCE(this->get_logger(), "pub_interval %4d", pub_interval_);

  if (!shapes_json_.empty())
  {
    if (!std::filesystem::exists(shapes_json_) || !std::filesystem::is_regular_file(shapes_json_))
      RCLCPP_ERROR(this->get_logger(), "Error: %s des not exist", shapes_json_.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Error: json not set");
  }
}
