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

  pub_object_costmap_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("object_costmap", 10);

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
  for(const auto &o: msg->objects){
      Object object;
      object.id = o.id;
      object.type = o.type;
      object.geometry.type = Geometry::LineString;
      for(const auto &p: o.geo_points){
        cv::Vec3d p_g(p.latitude, p.longitude, p.altitude);
        object.geometry.points.push_back(p_g);
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

    nav_msgs::msg::OccupancyGrid costmap;
    costmap.header.frame_id = "map";
    costmap.info.width = 100;  // Set your desired width
    costmap.info.height = 100; // Set your desired height
    costmap.info.resolution = 0.1; // Set your desired resolution
    costmap.data.resize(costmap.info.width * costmap.info.height, 0);
  
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
    descriptor.description = "mapimage folder";
    this->declare_parameter<std::string>("mapimage_folder", "/home/markus/Downloads/mapimage/", descriptor);
    //this->declare_parameter<std::string>("mapimage_folder", "", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "resolution m/pix";
    this->declare_parameter<double>("resolution", 1. / 5.0, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "map width [cells]";
    this->declare_parameter<int>("map_width", 1000, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "map height [cells]";
    this->declare_parameter<int>("map_height", 1000, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map";
    this->declare_parameter<double>("map_origin_x", 0, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map";
    this->declare_parameter<double>("map_origin_y", 0, descriptor);
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
  double latitude, longitude, altitude;
  this->get_parameter<std::string>("map_topic", map_topic_);
  RCLCPP_INFO(this->get_logger(), "map_topic: %s", map_topic_.c_str());
  this->get_parameter<std::string>("mapimage_folder", mapimage_folder_);
  RCLCPP_INFO(this->get_logger(), "mapimage_folder: %s", mapimage_folder_.c_str());
  this->get_parameter<std::string>("json_file", json_file_);
  RCLCPP_INFO(this->get_logger(), "json_file: %s", json_file_.c_str());
  this->get_parameter<double>("resolution", object_map_.info().resolution);
  this->get_parameter<int>("map_width", object_map_.info().size.width);
  this->get_parameter<int>("map_height", object_map_.info().size.height);
  this->get_parameter<double>("map_origin_x", object_map_.info().origin[0]);
  this->get_parameter<double>("map_origin_y", object_map_.info().origin[1]);
  this->get_parameter<double>("origin_latitude", latitude);
  this->get_parameter<double>("origin_longitude",longitude);
  this->get_parameter<double>("origin_altitude", altitude);
  if (mapimage_folder_.empty()){
    object_map_.init_map(latitude, longitude, altitude);
  } else {
    object_map_.init_map(mapimage_folder_);
  }
  RCLCPP_INFO(this->get_logger(), "%s", object_map_.info().info_map().c_str());
  RCLCPP_INFO(this->get_logger(), "%s", object_map_.info().info_geo().c_str());
}
