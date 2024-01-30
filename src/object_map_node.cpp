#include "tuw_object_map/object_map_node.hpp"
#include <GeographicLib/UTMUPS.hpp>

using std::placeholders::_1;

using namespace tuw_object_map;

ObjectMapNode::ObjectMapNode(const std::string & node_name)
: Node(node_name)
{
  declare_parameters();
  read_parameters();

  sub_map_ = create_subscription<tuw_object_map_msgs::msg::ObjectMap>(
    map_topic_, 10, std::bind(&ObjectMapNode::callback_object_map, this, _1));

  using namespace std::chrono_literals;
  timer_ = create_wall_timer(5000ms, std::bind(&ObjectMapNode::callback_timer, this));
}

void ObjectMapNode::callback_object_map(
  const tuw_object_map_msgs::msg::ObjectMap::SharedPtr msg)
{
  (void) msg;
  RCLCPP_INFO(this->get_logger(), "I received a map");
}

void ObjectMapNode::callback_timer() { 
  RCLCPP_INFO(this->get_logger(), "on_timer"); 
}

void ObjectMapNode::declare_parameters()
{
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "map topic";
    this->declare_parameter<std::string>("map_topic", "map", descriptor);
  }
  
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "resolution m/pix";
    this->declare_parameter<double>("resolution", 0.05, descriptor);
  }
}

void ObjectMapNode::read_parameters()
{
  this->get_parameter<std::string>("graph_file", map_topic_);
  this->get_parameter<double>("resolution", map_resolution_);
}
