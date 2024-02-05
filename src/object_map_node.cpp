#include "tuw_object_map/object_map_node.hpp"
#include <tuw_object_map_msgs/object_map_json.hpp>
#include <tuw_json/json.hpp>

using std::placeholders::_1;

using namespace tuw_object_map;

ObjectMapNode::ObjectMapNode(const std::string &node_name)
    : Node(node_name)
{
  declare_parameters();
  read_parameters();

  pub_occupancy_grid_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("object_costmap", 10);
  pub_occupancy_grid_img_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("object_img_map", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  if (json_file_.empty())
  {
    sub_map_ = create_subscription<tuw_object_map_msgs::msg::ObjectMap>(
        map_topic_, 10, std::bind(&ObjectMapNode::callback_object_map, this, _1));
  }
  else
  {
    tuw_object_map_msgs::msg::ObjectMap::SharedPtr map = std::make_shared<tuw_object_map_msgs::msg::ObjectMap>();
    tuw_json::fromJson(tuw_json::read(json_file_, "object_map"), *map);
    callback_object_map(map);
    callback_timer();
  }
  using namespace std::chrono_literals;
  timer_ = create_wall_timer(100ms, std::bind(&ObjectMapNode::callback_timer, this));
}

void ObjectMapNode::callback_object_map(
    const tuw_object_map_msgs::msg::ObjectMap::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "I received a map");

  for (const auto &o : msg->objects)
  {
    cv::Vec3d p0, p1;
    if (o.type == tuw_object_map_msgs::msg::Object::TYPE_PLANT_WINE_ROW)
    {
      if (o.geo_points.size() > 0)
      {
        p0 = cv::Vec3d(o.geo_points[0].latitude, o.geo_points[0].longitude, o.geo_points[0].altitude);
      }
      for (size_t i = 1; i < o.geo_points.size(); i++)
      {
        p1 = cv::Vec3d(o.geo_points[i].latitude, o.geo_points[i].longitude, o.geo_points[i].altitude);
        double enflation = o.enflation_radius[i];
        double bondary = o.bondary_radius[i];
        object_map_.line(p0, p1, bondary, enflation);
      }
    }
  }

  cv::Mat &img_src = object_map_.process();

  occupancy_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  occupancy_map_->header.frame_id = frame_object_map_;
  occupancy_map_->info.width = img_src.cols;                       // Set your desired width
  occupancy_map_->info.height = img_src.rows;                      // Set your desired height
  occupancy_map_->info.resolution = object_map_.info().resolution; // Set your desired resolution
  occupancy_map_->info.origin.position.x = object_map_.info().origin.x();
  occupancy_map_->info.origin.position.y = object_map_.info().origin.y() - img_src.rows * object_map_.info().resolution;
  occupancy_map_->info.origin.position.z = 0;
  occupancy_map_->data.resize(occupancy_map_->info.width * occupancy_map_->info.height, 0);

  cv::Mat_<int8_t> img_des = cv::Mat(img_src.size(), CV_8S, &occupancy_map_->data[0]);
  for (int r = 0; r < img_des.rows; r++)
  {
    for (int c = 0; c < img_des.cols; c++)
    {
      uint8_t &src = img_src.at<uint8_t>(r, c);
      int8_t &des = img_des.at<int8_t>(img_des.rows - r - 1, c);
      switch (src)
      {
      case ObjectMap::CELL_FREE:
        des = 0;
        break;
      case ObjectMap::CELL_OCCUPIED:
        des = 100;
        break;
      default:
        des = -1;
        break;
      }
    }
  }

  pub_occupancy_grid_map_->publish(*occupancy_map_);

  if (!object_map_.img_map().empty())
  {
    cv::Mat &img_src = object_map_.img_map();
    occupancy_img_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    occupancy_img_->header.frame_id = frame_satellit_map_;
    occupancy_img_->info = occupancy_map_->info;
    occupancy_img_->data.resize(occupancy_img_->info.width * occupancy_img_->info.height, 0);

    cv::Mat_<int8_t> img_des = cv::Mat(img_src.size(), CV_8S, &occupancy_img_->data[0]);
    for (int r = 0; r < img_des.rows; r++)
    {
      for (int c = 0; c < img_des.cols; c++)
      {
        cv::Vec<uint8_t, 3> &src = img_src.at<cv::Vec<uint8_t, 3>>(r, c);
        int8_t &des = img_des.at<int8_t>(img_des.rows - r - 1, c);
        des = (static_cast<int>(src[0]) + static_cast<int>(src[1]) + static_cast<int>(src[2])) / 3;
      }
    }
    pub_occupancy_grid_img_->publish(*occupancy_img_);
  }

  if (show_map_)
    object_map_.imshow(100);
  publish_transforms();
}

void ObjectMapNode::publish_transforms()
{
  {
    geometry_msgs::msg::TransformStamped tf_map;
    tf_map.header.stamp = this->get_clock()->now();
    tf_map.header.frame_id = "utm";
    tf_map.child_frame_id = frame_id_;
    tf_map.transform.translation.x = object_map_.info().utm()[0];
    tf_map.transform.translation.y = object_map_.info().utm()[1];
    tf_map.transform.translation.z = object_map_.info().utm()[2];
    tf_broadcaster_->sendTransform(tf_map);
  }
  {
    geometry_msgs::msg::TransformStamped tf_object_map;
    tf_object_map.header.stamp = this->get_clock()->now();
    tf_object_map.header.frame_id = "utm";
    tf_object_map.child_frame_id = frame_object_map_;
    tf_object_map.transform.translation.x = object_map_.info().utm()[0];
    tf_object_map.transform.translation.y = object_map_.info().utm()[1];
    tf_object_map.transform.translation.z = object_map_.info().utm()[2];
    tf_broadcaster_->sendTransform(tf_object_map);
  }

  if (!object_map_.img_map().empty())
  {
    geometry_msgs::msg::TransformStamped tf_satellit_map;
    tf_satellit_map.header.stamp = this->get_clock()->now();
    tf_satellit_map.header.frame_id = "utm";
    tf_satellit_map.child_frame_id = frame_satellit_map_;
    tf_satellit_map.transform.translation.x = object_map_.info().utm()[0];
    tf_satellit_map.transform.translation.y = object_map_.info().utm()[1];
    tf_satellit_map.transform.translation.z = object_map_.info().utm()[2];
    tf_broadcaster_->sendTransform(tf_satellit_map);
  }
}

void ObjectMapNode::callback_timer()
{
  // RCLCPP_INFO(this->get_logger(), "on_timer");

  publish_transforms();
  if (occupancy_img_)
  {
    pub_occupancy_grid_img_->publish(*occupancy_img_);
  }
  if (occupancy_map_)
  {
    pub_occupancy_grid_map_->publish(*occupancy_map_);
    if (show_map_)
      object_map_.imshow(1000);
  }
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
    descriptor.description = "shows the map in a opencv window";
    this->declare_parameter<bool>("show_map", true, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "frame_id";
    this->declare_parameter<std::string>("frame_id", "map", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "frame_object_map";
    this->declare_parameter<std::string>("frame_object_map", "object_map", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "frame_satellit_map";
    this->declare_parameter<std::string>("frame_satellit_map", "satellit_map", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "json_file";
    this->declare_parameter<std::string>("json_file", "", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "mapimage folder";
    this->declare_parameter<std::string>("mapimage_folder", "", descriptor);
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

void ObjectMapNode::read_parameters()
{
  double latitude, longitude, altitude;
  this->get_parameter<std::string>("map_topic", map_topic_);
  RCLCPP_INFO(this->get_logger(), "map_topic: %s", map_topic_.c_str());
  this->get_parameter<bool>("show_map", show_map_);
  RCLCPP_INFO(this->get_logger(), "show_map: %s", (show_map_ ? "true" : "false"));
  this->get_parameter<std::string>("frame_id", frame_id_);
  this->get_parameter<std::string>("frame_object_map_", frame_object_map_);
  this->get_parameter<std::string>("frame_satellit_map", frame_satellit_map_);
  RCLCPP_INFO(this->get_logger(), "frame_id_: %s, frame_object_map_: %s, frame_satellit_map: %s", frame_id_.c_str(), frame_object_map_.c_str(), frame_satellit_map_.c_str());
  this->get_parameter<std::string>("mapimage_folder", mapimage_folder_);
  this->get_parameter<std::string>("json_file", json_file_);
  RCLCPP_INFO(this->get_logger(), "json_file: %s", json_file_.c_str());
  this->get_parameter<double>("resolution", object_map_.info().resolution);
  this->get_parameter<int>("map_width", object_map_.info().size.width);
  this->get_parameter<int>("map_height", object_map_.info().size.height);
  this->get_parameter<double>("map_origin_x", object_map_.info().origin.x());
  this->get_parameter<double>("map_origin_y", object_map_.info().origin.y());
  this->get_parameter<double>("origin_latitude", latitude);
  this->get_parameter<double>("origin_longitude", longitude);
  this->get_parameter<double>("origin_altitude", altitude);
  if (mapimage_folder_.empty())
  {
    object_map_.init_map(latitude, longitude, altitude);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "mapimage_folder: %s", mapimage_folder_.c_str());
    RCLCPP_INFO(this->get_logger(), "using mapimage parameter, arguments are overwritten");
    object_map_.init_map(mapimage_folder_);
  }
  RCLCPP_INFO(this->get_logger(), "%s", object_map_.info().info_map().c_str());
  RCLCPP_INFO(this->get_logger(), "%s", object_map_.info().info_geo().c_str());
}
