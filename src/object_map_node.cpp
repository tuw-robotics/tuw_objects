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
  pub_pose_map_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_gps2map", 10);
  pub_pose_utm_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_gps2utm", 10);
  pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("plants", 10);

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
  timer_ = create_wall_timer(1000ms, std::bind(&ObjectMapNode::callback_timer, this));

  this->load_map_service_ = this->create_service<tuw_object_map_msgs::srv::LoadMap>("load_map", std::bind(&ObjectMapNode::callback_load_map, this, _1, _2));

  sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>("point_gps", 10, std::bind(&ObjectMapNode::callback_point_gps, this, _1));
}

void ObjectMapNode::load_map(const std::string &filename)
{
  tuw_object_map_msgs::msg::ObjectMap::SharedPtr map = std::make_shared<tuw_object_map_msgs::msg::ObjectMap>();
  tuw_json::fromJson(tuw_json::read(filename, "object_map"), *map);
  callback_object_map(map);
  callback_timer();
}

void ObjectMapNode::callback_object_map(
    const tuw_object_map_msgs::msg::ObjectMap::SharedPtr msg)
{
  if (object_map_msg_.objects == msg->objects)
  {
    RCLCPP_INFO(this->get_logger(), "I received a knwon map");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "I received a new map");
    object_map_msg_.objects = msg->objects;

    tuw::GeoMapMetaData info_;
    RCLCPP_INFO(this->get_logger(), "Updating map mansfen e.g. map origin and map size");
    cv::Vec3d utm_tl, utm_br; /// top left and bottom right
    int nr_of_points = 0;
    for (const auto &o : object_map_msg_.objects)
    {
      for (const auto &g : o.geo_points)
      {
        if (nr_of_points == 0)
        {
          info_.init(g.latitude, g.longitude, g.altitude);
          cv::Vec3d p_utm = info_.lla2utm(cv::Vec3d(g.latitude, g.longitude, g.altitude));
          utm_tl = p_utm; /// top left
          utm_br = p_utm; /// bottom right
        }
        cv::Vec3d p_utm = info_.lla2utm(cv::Vec3d(g.latitude, g.longitude, g.altitude));
        if (utm_tl[0] < p_utm[0])
          utm_tl[0] = p_utm[0];
        if (utm_tl[1] > p_utm[1])
          utm_tl[1] = p_utm[1];
        if (utm_tl[2] < p_utm[2])
          utm_tl[2] = p_utm[2];

        if (utm_br[0] > p_utm[0])
          utm_br[0] = p_utm[0];
        if (utm_br[1] < p_utm[1])
          utm_br[1] = p_utm[1];
        if (utm_br[2] > p_utm[2])
          utm_br[2] = p_utm[2];
        nr_of_points++;
      }
    }

    /// adding border to outer points
    utm_tl = utm_tl + cv::Vec3d(+map_border_, -map_border_, +map_border_);
    utm_br = utm_br + cv::Vec3d(-map_border_, +map_border_, -map_border_);

    std::shared_ptr<ObjectMap> object_map = std::make_shared<ObjectMap>();

    /// compute map size
    object_map->resolution = resolution_;
    object_map->size.width = fabs((utm_tl[0] - utm_br[0])) / resolution_;
    object_map->size.height = fabs((utm_tl[1] - utm_br[1])) / resolution_;

    /// compute origin depnding on the map offset
    /// ToDo check if the offset works
    cv::Vec3d g_origin = info_.utm2lla(utm_br + cv::Vec3d(object_map->origin.x(), object_map->origin.y(), 0));
    object_map->init(g_origin[0], g_origin[1], g_origin[2]);
    RCLCPP_INFO(this->get_logger(), "%s", object_map->info_map().c_str());
    RCLCPP_INFO(this->get_logger(), "%s", object_map->info_geo().c_str());

    object_map->draw(object_map_msg_);

    cv::Mat &img_src = object_map->mat();

    if (!debug_folder_.empty())
    {
      cv::imwrite(debug_folder_ + "object_map.png", img_src);
    }

    occupancy_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    occupancy_map_->header.frame_id = frame_map_;
    occupancy_map_->info.width = img_src.cols;     // Set your desired width
    occupancy_map_->info.height = img_src.rows;    // Set your desired height
    occupancy_map_->info.resolution = resolution_; // Set your desired resolution
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
    object_map_ = object_map;
  }
  pub_occupancy_grid_map_->publish(*occupancy_map_);

  if (show_map_)
    object_map_->imshow(100);

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
  }
  else
  {
    response->result = tuw_object_map_msgs::srv::LoadMap::Response::RESULT_UNDEFINED_FAILURE;
  }
}
void ObjectMapNode::publish_marker()
{
  if (!object_map_)
    return;
    
  if (!marker_msg_)
  {
    marker_msg_ = std::make_shared<visualization_msgs::msg::Marker>();
  }
  marker_msg_->header.frame_id = "utm";
  marker_msg_->header.stamp = this->now();
  marker_msg_->id = 0;
  marker_msg_->type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker_msg_->action = visualization_msgs::msg::Marker::ADD;
  marker_msg_->scale.x = 1.0;
  marker_msg_->scale.y = 1.0;
  marker_msg_->scale.z = 1.0;
  marker_msg_->color.r = 1.0;
  marker_msg_->color.g = 0.0;
  marker_msg_->color.b = 0.0;
  marker_msg_->color.a = 1.0;
  marker_msg_->points.clear();
  for (const auto &o : object_map_msg_.objects)
  {
    for (const auto &g : o.geo_points)
    {
      cv::Vec3d p_utm = object_map_->lla2utm(cv::Vec3d(g.latitude, g.longitude, g.altitude));
      geometry_msgs::msg::Point p;
      p.x = p_utm[0];
      p.y = p_utm[1];
      p.z = p_utm[2];
      marker_msg_->points.push_back(std::move(p));
    }
  }
  pub_marker_->publish(*marker_msg_);
}

void ObjectMapNode::publish_transforms()
{
  if (publish_tf_ && object_map_)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = frame_utm_;
    tf.child_frame_id = frame_map_;
    cv::Vec3d utm_bl = object_map_->map2utm(cv::Point(0, object_map_->size.height - 1));
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
  publish_marker();
  if (occupancy_map_)
  {
    pub_occupancy_grid_map_->publish(*occupancy_map_);
    if (show_map_)
      object_map_->imshow(1000);
  }
}

void ObjectMapNode::declare_parameters()
{
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "On true a tf from frame_utm to frame_map is published";
    this->declare_parameter<bool>("publish_tf", true, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "Name of the map frame, only need if publish_tf == true";
    this->declare_parameter<std::string>("frame_map", "object_map", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "Name of the utm frame, only need if publish_tf == true";
    this->declare_parameter<std::string>("frame_utm", "utm", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "Filename to load the object map from a json file if not set the node will wait for a msg on the topic";
    this->declare_parameter<std::string>("json_file", "", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "Resolution of the generated map [m/pix]";
    this->declare_parameter<double>("resolution", 0.1, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "Border on the created map [meter]";
    this->declare_parameter<double>("map_border", 10.0, descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "If set it stores debug information and images there";
    this->declare_parameter<std::string>("debug_folder", "/tmp/ros/object_map", descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = "Shows the map in a opencv window";
    this->declare_parameter<bool>("show_map", false, descriptor);
  }
}

void ObjectMapNode::read_parameters()
{
  this->get_parameter<std::string>("debug_folder", debug_folder_);
  this->get_parameter<bool>("show_map", show_map_);
  this->get_parameter<std::string>("frame_map", frame_map_);
  this->get_parameter<std::string>("frame_utm", frame_utm_);
  this->get_parameter<std::string>("json_file", json_file_);
  this->get_parameter<double>("resolution", resolution_);
  this->get_parameter<double>("map_border", map_border_);
  this->get_parameter<bool>("publish_tf", publish_tf_);

  RCLCPP_INFO(this->get_logger(), "debug_folder: '%s', if set it stores debug images there!", debug_folder_.c_str());
  RCLCPP_INFO(this->get_logger(), "show_map: %s", (show_map_ ? "true" : "false"));
  RCLCPP_INFO(this->get_logger(), "json_file: %s", json_file_.c_str());
  RCLCPP_INFO(this->get_logger(), "publish_tf: %s",
              (publish_tf_ ? " true: frame_utm -> frame_map is published" : " false: not tf is published"));
  RCLCPP_INFO(this->get_logger(), "frame_map: %s, frame_utm: %s", frame_map_.c_str(), frame_utm_.c_str());
}

void ObjectMapNode::callback_point_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "callback_point_gps");
  if (object_map_)
  {
    cv::Vec3d position_utm = object_map_->lla2utm(cv::Vec3d(msg->latitude, msg->longitude, msg->altitude));
    geometry_msgs::msg::PoseStamped pose_utm;
    pose_utm.header.frame_id = frame_utm_;
    pose_utm.header.stamp = this->get_clock()->now();
    pose_utm.pose.position.x = position_utm[0];
    pose_utm.pose.position.y = position_utm[1];
    pose_utm.pose.position.z = position_utm[2];
    pose_utm.pose.orientation.x = 0.707;
    pose_utm.pose.orientation.y = 0;
    pose_utm.pose.orientation.z = 0;
    pose_utm.pose.orientation.w = 0.707;
    pub_pose_utm_->publish(pose_utm);

    cv::Vec3d position_map = object_map_->utm2world(position_utm);
    geometry_msgs::msg::PoseStamped pose_map;
    pose_map.header.frame_id = frame_map_;
    pose_map.header.stamp = this->get_clock()->now();
    pose_map.pose.position.x = position_map[0];
    pose_map.pose.position.y = position_map[1];
    pose_map.pose.position.z = position_map[2];
    pose_map.pose.orientation.x = 0.707;
    pose_map.pose.orientation.y = 0;
    pose_map.pose.orientation.z = 0;
    pose_map.pose.orientation.w = 0.707;
    pub_pose_map_->publish(pose_map);
  }
}