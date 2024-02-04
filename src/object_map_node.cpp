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
    callback_timer();
  }
  using namespace std::chrono_literals;
  timer_ = create_wall_timer(5000ms, std::bind(&ObjectMapNode::callback_timer, this));

}

void ObjectMapNode::callback_object_map(
  const tuw_object_map_msgs::msg::ObjectMap::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "I received a map");

  for(const auto &o: msg->objects){
      cv::Vec3d p0, p1;
      if(o.type == tuw_object_map_msgs::msg::Object::TYPE_PLANT_WINE_ROW){
        if(o.geo_points.size() > 0){
          p0 = cv::Vec3d (o.geo_points[0].latitude, o.geo_points[0].longitude, o.geo_points[0].altitude);
        }
        for(size_t i = 1; i < o.geo_points.size(); i++){
          p1 = cv::Vec3d (o.geo_points[i].latitude, o.geo_points[i].longitude, o.geo_points[i].altitude);
          double enflation = o.enflation_radius[i];
          double bondary = o.bondary_radius[i];
          object_map_.line(p0, p1, bondary, enflation);
        }
      }
  }
  
  cv::Mat &img_costmap = object_map_.process();

  nav_msgs::msg::OccupancyGrid costmap;
  costmap.header.frame_id = frame_id_;
  costmap.info.width = img_costmap.cols;  // Set your desired width
  costmap.info.height = img_costmap.rows; // Set your desired height
  costmap.info.resolution = 0.1; // Set your desired resolution
  costmap.data.resize(costmap.info.width * costmap.info.height, 0);
  for(size_t i = 0; i < costmap.data.size(); i++){
    costmap.data[i] = img_costmap.data[i];
  }
  pub_object_costmap_->publish(costmap);
  
}

void ObjectMapNode::callback_timer() { 
  RCLCPP_INFO(this->get_logger(), "on_timer"); 
  object_map_.imshow();
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
    descriptor.description = "frame_id";
    this->declare_parameter<std::string>("frame_id", "map", descriptor);
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
  this->get_parameter<std::string>("frame_id", frame_id_);
  RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
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
