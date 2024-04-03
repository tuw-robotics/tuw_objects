#include "tuw_object_map/object_map_node.hpp"
#include <tuw_object_map_msgs/object_map_json.hpp>
#include <tuw_json/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>

using std::placeholders::_1;
using std::placeholders::_2;

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
  pub_occupancy_grid_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("object_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  if (json_file_.empty())
  {
    sub_map_ = create_subscription<tuw_object_map_msgs::msg::ObjectMap>("objects", 10, std::bind(&ObjectMapNode::callback_object_map, this, _1));
  }
  else
  {
    load_map(json_file_);
  }

  using namespace std::chrono_literals;
  timer_ = create_wall_timer(100ms, std::bind(&ObjectMapNode::callback_timer, this));

  this->load_map_service_ = this->create_service<tuw_object_map_msgs::srv::LoadMap>("load_map", std::bind(&ObjectMapNode::callback_load_map, this, _1, _2));
}

void ObjectMapNode::load_map(const std::string &filename) {
    tuw_object_map_msgs::msg::ObjectMap::SharedPtr map = std::make_shared<tuw_object_map_msgs::msg::ObjectMap>();
    tuw_json::fromJson(tuw_json::read(filename, "object_map"), *map);
    callback_object_map(map);
    callback_timer();
}

void ObjectMapNode::callback_object_map(
    const tuw_object_map_msgs::msg::ObjectMap::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "I received a map");


  tuw::GeoMapMetaData info_;
  RCLCPP_INFO(this->get_logger(), "Updating map mansfen e.g. map origin and map size");
  cv::Vec3d utm_tl, utm_br; /// top left and bottom right
  int nr_of_points = 0;
  for (const auto &o : msg->objects){
    for (const auto &g : o.geo_points){
      if(nr_of_points == 0){
        info_.init(g.latitude, g.longitude, g.altitude);
        cv::Vec3d p_utm = info_.lla2utm(cv::Vec3d(g.latitude, g.longitude, g.altitude));
        utm_tl = p_utm; /// top left
        utm_br = p_utm;  /// bottom right
      } 
      cv::Vec3d p_utm = info_.lla2utm(cv::Vec3d(g.latitude, g.longitude, g.altitude));
      if(utm_tl[0] < p_utm[0]) utm_tl[0] = p_utm[0];
      if(utm_tl[1] > p_utm[1]) utm_tl[1] = p_utm[1];
      if(utm_tl[2] < p_utm[2]) utm_tl[2] = p_utm[2];

      if(utm_br[0] > p_utm[0]) utm_br[0] = p_utm[0];
      if(utm_br[1] < p_utm[1]) utm_br[1] = p_utm[1];
      if(utm_br[2] > p_utm[2]) utm_br[2] = p_utm[2];
      nr_of_points++;
    }
  }

  /// adding border to outer points
  utm_tl = utm_tl + cv::Vec3d(+map_border_, -map_border_, +map_border_);
  utm_br = utm_br + cv::Vec3d(-map_border_, +map_border_, -map_border_);

  /// compute map size
  object_map_.size.width = fabs((utm_tl[0] - utm_br[0]))/object_map_.resolution;
  object_map_.size.height = fabs((utm_tl[1] - utm_br[1]))/object_map_.resolution;

  /// compute origin depnding on the map offset 
  /// ToDo check if the offset works
  cv::Vec3d g_origin = info_.utm2lla(utm_br + cv::Vec3d(object_map_.origin.x(), object_map_.origin.y(), 0));
  object_map_.init(g_origin[0], g_origin[1], g_origin[2]);
  RCLCPP_INFO(this->get_logger(), "%s", object_map_.info_map().c_str());
  RCLCPP_INFO(this->get_logger(), "%s", object_map_.info_geo().c_str());

  

  draw(msg);

  cv::Mat &img_src = object_map_.process();

  if (!debug_folder_.empty())
  {
    cv::imwrite(debug_folder_ + "object_map.png", img_src);
  }

  occupancy_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  occupancy_map_->header.frame_id = frame_map_;
  occupancy_map_->info.width = img_src.cols;                       // Set your desired width
  occupancy_map_->info.height = img_src.rows;                      // Set your desired height
  occupancy_map_->info.resolution = object_map_.resolution; // Set your desired resolution
  occupancy_map_->info.origin.position.x = 0;
  occupancy_map_->info.origin.position.y = 0;
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

  if (show_map_)
    object_map_.imshow(100);
  publish_transforms();
}

void ObjectMapNode::callback_load_map(const std::shared_ptr<tuw_object_map_msgs::srv::LoadMap::Request> request, std::shared_ptr<tuw_object_map_msgs::srv::LoadMap::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "LoadMap service called!");

  if (!request->map_url.empty())
  {
    load_map(request->map_url);
    response->result = tuw_object_map_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
    response->map = *occupancy_map_;
  } else {
      response->result = tuw_object_map_msgs::srv::LoadMap::Response::RESULT_UNDEFINED_FAILURE;
  }
}

void ObjectMapNode::publish_transforms()
{
  if (publish_tf_)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = frame_utm_;
    tf.child_frame_id = frame_map_;
    cv::Vec3d utm_bl = object_map_.map2utm(cv::Point(0, object_map_.size.height-1));
    tf.transform.translation.x = utm_bl[0];
    tf.transform.translation.y = utm_bl[1];
    tf.transform.translation.z = utm_bl[2];
    RCLCPP_INFO_ONCE(this->get_logger(), "publish TF: frame_id: %s, child_frame_id: %s", tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
    tf_broadcaster_->sendTransform(tf);
  }
}

void ObjectMapNode::callback_timer()
{
  // RCLCPP_INFO(this->get_logger(), "on_timer");

  publish_transforms();
  if (occupancy_map_)
  {
    pub_occupancy_grid_map_->publish(*occupancy_map_);
    if (show_map_)
      object_map_.imshow(1000);
  }
}

void ObjectMapNode::draw(tuw_object_map_msgs::msg::ObjectMap::SharedPtr msg)
{

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
    else if ((o.type == tuw_object_map_msgs::msg::Object::TYPE_OBSTACLE_TREE))
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
  object_map_.img_costmap()(0, 0) = ObjectMap::CELL_OCCUPIED;
  object_map_.img_costmap()(object_map_.img_costmap().rows - 1, 0) = ObjectMap::CELL_OCCUPIED;
  object_map_.img_costmap()(object_map_.img_costmap().rows - 1, object_map_.img_costmap().cols - 1) = ObjectMap::CELL_OCCUPIED;
  object_map_.img_costmap()(0, object_map_.img_costmap().cols - 1) = ObjectMap::CELL_OCCUPIED;
}

void ObjectMapNode::declare_parameters()
{
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "frame_map name of the map frame, only need if publish_tf == true";
    this->declare_parameter<std::string>("frame_map", "object_map", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "frame_utm name of the utm frame, only need if publish_tf == true";
    this->declare_parameter<std::string>("frame_utm", "utm", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "json_file";
    this->declare_parameter<std::string>("json_file", "", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "resolution m/pix";
    this->declare_parameter<double>("resolution", 1. / 5.0, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "border in meters, on the created map";
    this->declare_parameter<double>("map_border", 10, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "on true a tf from frame_utm to frame_map is published";
    this->declare_parameter<bool>("publish_tf", true, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "debug folder, if set it stores debug information and images there";
    this->declare_parameter<std::string>("debug_folder", "/tmp/ros/object_map", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "shows the map in a opencv window";
    this->declare_parameter<bool>("show_map", true, descriptor);
  }
}

void ObjectMapNode::read_parameters()
{
  this->get_parameter<std::string>("debug_folder", debug_folder_);
  this->get_parameter<bool>("show_map", show_map_);
  this->get_parameter<std::string>("frame_map", frame_map_);
  this->get_parameter<std::string>("frame_utm", frame_utm_);
  this->get_parameter<std::string>("json_file", json_file_);
  this->get_parameter<double>("resolution", object_map_.resolution);
  this->get_parameter<double>("map_border", map_border_);
  this->get_parameter<bool>("publish_tf", publish_tf_);
  
  RCLCPP_INFO(this->get_logger(), "debug_folder: '%s', if set it stores debug images there!", debug_folder_.c_str());
  RCLCPP_INFO(this->get_logger(), "show_map: %s", (show_map_ ? "true" : "false"));
  RCLCPP_INFO(this->get_logger(), "json_file: %s", json_file_.c_str());
  RCLCPP_INFO(this->get_logger(), "publish_tf: %s",
              (publish_tf_ ? " true: frame_utm -> frame_map is published" : " false: not tf is published"));
  RCLCPP_INFO(this->get_logger(), "frame_map: %s, frame_utm: %s", frame_map_.c_str(), frame_utm_.c_str());
}
