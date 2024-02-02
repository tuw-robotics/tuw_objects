#include "tuw_object_map/object_map_node.hpp"
#include <tuw_object_map_msgs/object_map_json.hpp>
#include <tuw_json/json.hpp>

using std::placeholders::_1;

using namespace tuw_object_map;

ObjectMapNode::ObjectMapNode(const std::string & node_name)
: Node(node_name)
{
  declare_parameters();
  read_parameters();

  if(json_file_.empty()){
  sub_map_ = create_subscription<tuw_object_map_msgs::msg::ObjectMap>(
    map_topic_, 10, std::bind(&ObjectMapNode::callback_object_map, this, _1));
  } else {
    tuw_object_map_msgs::msg::ObjectMap::SharedPtr map = std::make_shared<tuw_object_map_msgs::msg::ObjectMap>();
    tuw_json::fromJson(tuw_json::read(json_file_, "object_map"), *map);
    callback_object_map(map);
  }
  using namespace std::chrono_literals;
  timer_ = create_wall_timer(5000ms, std::bind(&ObjectMapNode::callback_timer, this));

}

void ObjectMapNode::callback_object_map(
  const tuw_object_map_msgs::msg::ObjectMap::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "I received a map");
  std::vector<Object> objects;
  object_map_.set_origin(origin_lla_, map_resolution_);
  for(const auto &o: msg->objects){
      Object object;
      object.id = o.id;
      object.type = o.type;
      object.geometry.type = Geometry::LineString;
      for(const auto &p: o.geo_points){
        object.geometry.points.push_back(cv::Vec3d(p.latitude, p.longitude, p.altitude));
      }
      for(const auto &v: o.enflation_radius){
        object.geometry.enflation.push_back(v);
      }
      for(const auto &v: o.bondary_radius){
        object.geometry.bondary.push_back(v);
      }
      objects.push_back(std::move(object));
  }
  object_map_.process(objects);
}

void ObjectMapNode::callback_timer() { 
  RCLCPP_INFO(this->get_logger(), "on_timer"); 
}

void ObjectMapNode::declare_parameters()
{
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "map topic";
    this->declare_parameter<std::string>("map_topic", "object_map", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "json_file";
    this->declare_parameter<std::string>("json_file", "", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "resolution m/pix";
    this->declare_parameter<double>("resolution", 0.05, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "origin latitude";
    this->declare_parameter<double>("origin_latitude", 46.8015409, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "origin longitude";
    this->declare_parameter<double>("origin_longitude", 15.8382641, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "origin altitude";
    this->declare_parameter<double>("origin_altitude", 338.917, descriptor);
  }
}

void ObjectMapNode::read_parameters()
{
  this->get_parameter<std::string>("map_topic", map_topic_);
  RCLCPP_INFO(this->get_logger(), "map_topic: %s", map_topic_.c_str());
  this->get_parameter<std::string>("json_file", json_file_);
  RCLCPP_INFO(this->get_logger(), "json_file: %s", json_file_.c_str());
  this->get_parameter<double>("resolution", map_resolution_);
  RCLCPP_INFO(this->get_logger(), "resolution: %f", map_resolution_);
  this->get_parameter<double>("origin_latitude", origin_lla_[0]);
  this->get_parameter<double>("origin_longitude", origin_lla_[1]);
  this->get_parameter<double>("origin_altitude", origin_lla_[2]);
  RCLCPP_INFO(this->get_logger(), "origin: %f, %f, %f", origin_lla_[0], origin_lla_[1], origin_lla_[2]);
}
