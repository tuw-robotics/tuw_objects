#include "tuw_objects_server/objects_server_node.hpp"
#include <tuw_object_map_msgs/objects_json.hpp>
#include <tuw_json/json.hpp>
#include <filesystem>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using namespace tuw_objects;

ObjectsServerNode::ObjectsServerNode(const std::string &node_name)
    : Node(node_name)
{
  declare_parameters();
  read_parameters();

  read_objects(objects_json_);
  pub_objects_ = this->create_publisher<tuw_object_map_msgs::msg::Objects>(topic_name_objects_, 10);
  publish_objects();

  if(pub_interval_ > 0){
    using namespace std::chrono_literals;
    timer_ = create_wall_timer(1000ms * pub_interval_, std::bind(&ObjectsServerNode::callback_timer, this));
  } else {
    RCLCPP_INFO(get_logger(), "The objects are only published once becaue of pub_interval:=0. The node is waiting for service requests");
  }
  // Create a service that provides the occupancy grid
  srv_objects_ = create_service<tuw_object_map_msgs::srv::GetObjects>(
    service_name_objects_,
    std::bind(&ObjectsServerNode::callback_get_objects, this, _1, _2, _3));

  srv_publish_ = create_service<std_srvs::srv::Trigger>(
    service_name_publish_,
    std::bind(&ObjectsServerNode::callback_publish, this, _1, _2, _3));
}

void ObjectsServerNode::read_objects(const std::string &filename)
{
  objects_ = std::make_shared<tuw_object_map_msgs::msg::Objects>();
  tuw_json::fromJson(tuw_json::read(filename, "objects"), *objects_);
  if(!frame_id_.empty()){
    objects_->header.frame_id =  frame_id_;
    objects_->header.stamp = this->get_clock()->now();
  }
  if(objects_->objects.empty()){
    RCLCPP_ERROR(this->get_logger(), "No objects in json. Check tag name!");
    return;
  }
}

void ObjectsServerNode::callback_get_objects(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<tuw_object_map_msgs::srv::GetObjects::Request>/*request*/,
  std::shared_ptr<tuw_object_map_msgs::srv::GetObjects::Response> response)
{
  RCLCPP_INFO(get_logger(), "Handling GetMap request");
  response->map = *objects_;
}

void ObjectsServerNode::callback_publish(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "Handling publish request");
  if(objects_){
    response->success = true;
    publish_objects();
  }
}

void ObjectsServerNode::callback_timer()
{
  publish_objects();
}


void ObjectsServerNode::publish_objects()
{
  if(!objects_) return;
  RCLCPP_INFO(this->get_logger(), "publish_objects");
  pub_objects_->publish(*objects_);
}

void ObjectsServerNode::declare_parameters()
{
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "Name of the frame";
    this->declare_parameter<std::string>("frame_id", "WGS84", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "Filename to load the objects from a json file if not set the node will wait for a msg on the topic";
    this->declare_parameter<std::string>("objects_json", "", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "publishing interval in seconds. If 0 or less, the graph is published once.";
    this->declare_parameter<int>("pub_interval", 10, descriptor);
  }
}

void ObjectsServerNode::read_parameters()
{
  this->get_parameter<std::string>("frame_id", frame_id_);
  RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
  this->get_parameter<std::string>("objects_json", objects_json_);
  RCLCPP_INFO(this->get_logger(), "objects_json: %s", objects_json_.c_str());
  this->get_parameter<int>("pub_interval", pub_interval_);
  RCLCPP_INFO_ONCE(this->get_logger(), "pub_interval %4d", pub_interval_);

  if (!objects_json_.empty())
  {
    if (!std::filesystem::exists(objects_json_) || !std::filesystem::is_regular_file(objects_json_))
      RCLCPP_ERROR(this->get_logger(), "Error: %s des not exist", objects_json_.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Error: objects_json not set");
  }
}
