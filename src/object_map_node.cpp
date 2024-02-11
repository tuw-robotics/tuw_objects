#include "tuw_object_map/object_map_node.hpp"
#include <tuw_object_map_msgs/object_map_json.hpp>
#include <tuw_json/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>

using std::placeholders::_1;

using namespace tuw_object_map;

ObjectMapNode::ObjectMapNode(const std::string &node_name)
    : Node(node_name)
{
  declare_parameters();
  read_parameters();

  if (!debug_folder_.empty())
  {
    if (debug_folder_.back() != '/') 
      debug_folder_ += '/';
    if (!std::filesystem::is_directory(debug_folder_) || !std::filesystem::exists(debug_folder_))
      std::filesystem::create_directories(debug_folder_); // create src folder
  }
  pub_occupancy_grid_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("object_costmap", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_occupancy_grid_img_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("object_img_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

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
  
  draw(msg);

  cv::Mat &img_src = object_map_.process();

  if (!debug_folder_.empty())
  {
    cv::imwrite(debug_folder_ + "object_map.png", img_src);
  }

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
  if (publish_utm_)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = frame_utm_;
    tf.child_frame_id = frame_map_;
    tf.transform.translation.x = object_map_.info().utm()[0];
    tf.transform.translation.y = object_map_.info().utm()[1];
    tf.transform.translation.z = object_map_.info().utm()[2];
    RCLCPP_INFO_ONCE(this->get_logger(), "publish TF: frame_id: %s, child_frame_id: %s", tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
    tf_broadcaster_->sendTransform(tf);
  }
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = (publish_utm_ ? frame_utm_ : frame_map_);
    tf.child_frame_id = frame_object_map_;
    if (publish_utm_)
    {
      tf.transform.translation.x = object_map_.info().utm()[0];
      tf.transform.translation.y = object_map_.info().utm()[1];
      tf.transform.translation.z = object_map_.info().utm()[2];
    }
    RCLCPP_INFO_ONCE(this->get_logger(), "publish TF: frame_id: %s, child_frame_id: %s", tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
    tf_broadcaster_->sendTransform(tf);
  }

  if (!object_map_.img_map().empty())
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = (publish_utm_ ? frame_utm_ : frame_map_);
    tf.child_frame_id = frame_satellit_map_;
    if (publish_utm_)
    {
      tf.transform.translation.x = object_map_.info().utm()[0];
      tf.transform.translation.y = object_map_.info().utm()[1];
      tf.transform.translation.z = object_map_.info().utm()[2];
    }
    tf_broadcaster_->sendTransform(tf);
    RCLCPP_INFO_ONCE(this->get_logger(), "publish TF: frame_id: %s, child_frame_id: %s", tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
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

void ObjectMapNode::draw(tuw_object_map_msgs::msg::ObjectMap::SharedPtr msg){

  /// Frist draw free space
  for (const auto &o : msg->objects)
  {
    cv::Vec3d p0, p1;
    if (o.type == tuw_object_map_msgs::msg::Object::TYPE_PLANT_WINE_ROW)
    {
      if (o.geo_points.size() > 0)
      {
        p0 = cv::Vec3d(o.geo_points[0].latitude, o.geo_points[0].longitude, o.geo_points[0].altitude);
        object_map_.line(p0, p0, ObjectMap::CELL_FREE, o.bondary_radius[0]);
      }
      for (size_t i = 1; i < o.geo_points.size(); i++)
      {
        p1 = cv::Vec3d(o.geo_points[i].latitude, o.geo_points[i].longitude, o.geo_points[i].altitude);
        object_map_.line(p0, p1, ObjectMap::CELL_FREE, o.bondary_radius[i]);
        p0 = p1;
      }
    }
    else if ((o.type == tuw_object_map_msgs::msg::Object::TYPE_TRANSIT) ||
             (o.type == tuw_object_map_msgs::msg::Object::TYPE_TRANSIT_STREET) ||
             (o.type == tuw_object_map_msgs::msg::Object::TYPE_TRANSIT_GRAVEL))
    {
      if (o.geo_points.size() > 0)
      {
        p0 = cv::Vec3d(o.geo_points[0].latitude, o.geo_points[0].longitude, o.geo_points[0].altitude);
        object_map_.line(p0, p0, ObjectMap::CELL_FREE, o.bondary_radius[0]);
      }
      for (size_t i = 1; i < o.geo_points.size(); i++)
      {
        p1 = cv::Vec3d(o.geo_points[i].latitude, o.geo_points[i].longitude, o.geo_points[i].altitude);
        object_map_.line(p0, p1, ObjectMap::CELL_FREE, o.bondary_radius[i]);
        p0 = p1;
      }
    }
  }
  /// Frist draw occupied space
  for (const auto &o : msg->objects)
  {
    cv::Vec3d p0, p1;
    if (o.type == tuw_object_map_msgs::msg::Object::TYPE_PLANT_WINE_ROW)
    {
      if (o.geo_points.size() > 0)
      {
        p0 = cv::Vec3d(o.geo_points[0].latitude, o.geo_points[0].longitude, o.geo_points[0].altitude);
        object_map_.line(p0, p0, ObjectMap::CELL_OCCUPIED, o.enflation_radius[0]);
      }
      for (size_t i = 1; i < o.geo_points.size(); i++)
      {
        p1 = cv::Vec3d(o.geo_points[i].latitude, o.geo_points[i].longitude, o.geo_points[i].altitude);
        object_map_.line(p0, p1, ObjectMap::CELL_OCCUPIED, o.enflation_radius[i]);
        p0 = p1;
      }
    }
    else if ((o.type == tuw_object_map_msgs::msg::Object::TYPE_OBSTACLE_TREE) )
    {
      cv::Vec3d p0, p1;
      if (o.geo_points.size() > 0)
      {
        p0 = cv::Vec3d(o.geo_points[0].latitude, o.geo_points[0].longitude, o.geo_points[0].altitude);
        object_map_.line(p0, p0, ObjectMap::CELL_OCCUPIED, o.enflation_radius[0]);

      }
      for (size_t i = 1; i < o.geo_points.size(); i++)
      {
        p1 = cv::Vec3d(o.geo_points[i].latitude, o.geo_points[i].longitude, o.geo_points[i].altitude);
        object_map_.line(p0, p1, ObjectMap::CELL_OCCUPIED, o.enflation_radius[i]);
        p0 = p1;
      }
    }
  }
  /// fill corners (for stage map)
  object_map_.img_costmap()(0,0) = ObjectMap::CELL_OCCUPIED;
  object_map_.img_costmap()(object_map_.img_costmap().rows-1,0) = ObjectMap::CELL_OCCUPIED;
  object_map_.img_costmap()(object_map_.img_costmap().rows-1,object_map_.img_costmap().cols-1) = ObjectMap::CELL_OCCUPIED;
  object_map_.img_costmap()(0,object_map_.img_costmap().cols-1) = ObjectMap::CELL_OCCUPIED;
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
    descriptor.description = "frame_map";
    this->declare_parameter<std::string>("frame_map", "map", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "frame_utm only need if publish_utm == true";
    this->declare_parameter<std::string>("frame_utm", "utm", descriptor);
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
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "on true the maps are published in relative to utm, otherwise with a zero link to frame_map";
    this->declare_parameter<bool>("publish_utm", false, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "debug folder, if set it stores debug information and images there";
    this->declare_parameter<std::string>("debug_folder", "/tmp/ros/object_map", descriptor);
  }
}

void ObjectMapNode::read_parameters()
{
  double latitude, longitude, altitude;
  this->get_parameter<std::string>("debug_folder", debug_folder_);
  RCLCPP_INFO(this->get_logger(), "debug_folder: '%s', if set it stores debug images there!", debug_folder_.c_str());
  this->get_parameter<std::string>("map_topic", map_topic_);
  RCLCPP_INFO(this->get_logger(), "map_topic: %s", map_topic_.c_str());
  this->get_parameter<bool>("show_map", show_map_);
  RCLCPP_INFO(this->get_logger(), "show_map: %s", (show_map_ ? "true" : "false"));
  this->get_parameter<std::string>("frame_map", frame_map_);
  this->get_parameter<std::string>("frame_utm", frame_utm_);
  this->get_parameter<std::string>("frame_object_map", frame_object_map_);
  this->get_parameter<std::string>("frame_satellit_map", frame_satellit_map_);
  RCLCPP_INFO(this->get_logger(), "frame_map: %s, frame_utm: %s, frame_object_map: %s, frame_satellit_map: %s",
              frame_map_.c_str(), frame_utm_.c_str(), frame_object_map_.c_str(), frame_satellit_map_.c_str());
  RCLCPP_INFO(this->get_logger(), "publish_utm: %s",
              (publish_utm_ ? " true -> maps are published in utm" : " false -> maps are published in map"));
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
  this->get_parameter<bool>("publish_utm", publish_utm_);
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
