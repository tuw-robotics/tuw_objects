#include "tuw_object_map/object_map_node.hpp"
#include <tuw_object_map_msgs/objects_json.hpp>
#include <tuw_json/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using namespace tuw_object_map;

ObjectMapNode::ObjectMapNode(const std::string &node_name)
    : Node(node_name)
{
  declare_parameters();
  read_static_parameters();
  read_dynamic_parameters();
  broadcaster_utm_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  if (!debug_root_folder_.empty())
  {
    if (debug_root_folder_.back() != '/')
      debug_root_folder_ += '/';
    debug_dest_folder_ = debug_root_folder_ + get_name() + "/";
    if (!std::filesystem::is_directory(debug_dest_folder_) || !std::filesystem::exists(debug_dest_folder_))
      std::filesystem::create_directories(debug_dest_folder_); // create debug destination folder for this node
    RCLCPP_INFO(this->get_logger(), "debug_dest_folder: %s", debug_dest_folder_.c_str());
  }

  pub_occupancy_grid_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(topic_name_map_to_provide_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_objects_on_map_ = this->create_publisher<tuw_object_map_msgs::msg::Objects>(topic_name_objects_to_provide_, 10);
  sub_object_map_ = create_subscription<tuw_object_map_msgs::msg::Objects>(topic_name_objects_to_subscribe_, 10, std::bind(&ObjectMapNode::callback_object_map, this, _1));

  service_objects_reqeust();

  if(pub_interval_ > 0){
    timer_ = create_wall_timer(std::chrono::milliseconds(1000) * pub_interval_, std::bind(&ObjectMapNode::on_timer, this));
  }
}

void ObjectMapNode::on_timer()
{
  RCLCPP_INFO(this->get_logger(), "on_timer");
  publish_map();
  publish_marker();
  publish_transforms_utm_map();
}
void ObjectMapNode::services_init_providors()
{
  RCLCPP_INFO(this->get_logger(), "services_init_providors");
  if (!srv_occ_map_)
  {
    // Create a service that provides the occupancy grid
    srv_occ_map_ = create_service<nav_msgs::srv::GetMap>(
        service_name_map_to_provide_,
        std::bind(&ObjectMapNode::callback_get_occ_map, this, _1, _2, _3));
  }

  if (!srv_object_map_)
  {
    // Create a service that provides the object map
    srv_object_map_ = create_service<tuw_object_map_msgs::srv::GetObjects>(
        service_name_objects_to_provide_,
        std::bind(&ObjectMapNode::callback_get_objects, this, _1, _2, _3));
  }
}
void ObjectMapNode::service_objects_reqeust()
{

  auto client = this->create_client<tuw_object_map_msgs::srv::GetObjects>(
      service_name_objects_to_call_);

  // Wait for the service to be available
  while (!client->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service GetObjects not available, waiting again ... ");
  }
  // Create a request
  auto request = std::make_shared<tuw_object_map_msgs::srv::GetObjects::Request>();

  auto result_future = client->async_send_request(request);

  // Wait for the result
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service map");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Successfuly call service map");
  auto result = result_future.get();
  process_objects(result->map);
}

void ObjectMapNode::callback_object_map(
    const tuw_object_map_msgs::msg::Objects::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "callback_objects");
  process_objects(*msg);
}

void ObjectMapNode::process_objects(const tuw_object_map_msgs::msg::Objects &msg)
{

  std::scoped_lock lock(lock_);
  if (msg_objects_received_ && (*msg_objects_received_ == msg) && (read_dynamic_parameters() == false))
  {
    RCLCPP_INFO(this->get_logger(), "received objects allready");
    return;
  }
  if(msg.objects.empty()){
    RCLCPP_INFO(this->get_logger(), "No objects in received msg");
    return;
  }
  msg_objects_received_ = std::make_shared<tuw_object_map_msgs::msg::Objects>(msg);

  auto msg_objects_tmp = std::make_shared<tuw_object_map_msgs::msg::Objects>(msg);

  RCLCPP_INFO(this->get_logger(), "I received a new map");

  static bool file_written = false;
  if (!file_written && !debug_dest_folder_.empty())
  {
    file_written = true;
    std::string json_file(debug_dest_folder_ + "objects.json");
    RCLCPP_INFO(this->get_logger(), "writing debug json file to: %s", json_file.c_str());
    tuw_json::write(json_file, "objects", tuw_json::toJson(*msg_objects_received_));
  }

  int utm_zone;
  bool utm_northp = true;
  cv::Vec3d p_utm;
  RCLCPP_INFO(this->get_logger(), "Updating map mansfen e.g. map origin and map size");
  cv::Vec3d utm_tr, utm_bl; /// top right and bottom left
  double gamma, k;
  int nr_of_points = 0;
  for (const auto &o : msg_objects_tmp->objects)
  {
    for (const auto &g : o.geo_points)
    {
      if (nr_of_points == 0)
      {
        cv::Vec3d p_lla(g.latitude, g.longitude, g.altitude);
        GeographicLib::UTMUPS::Forward(g.latitude, g.longitude, utm_zone, utm_northp, p_utm[0], p_utm[1], gamma, k);
        p_utm[2] = g.altitude;
        utm_tr = p_utm; /// top right

        utm_bl = p_utm; /// bottom left
      }
      GeographicLib::UTMUPS::Forward(g.latitude, g.longitude, utm_zone, utm_northp, p_utm[0], p_utm[1], gamma, k);
      p_utm[2] = g.altitude;
      if (utm_tr[0] < p_utm[0])
        utm_tr[0] = p_utm[0];
      if (utm_tr[1] < p_utm[1])
        utm_tr[1] = p_utm[1];
      if (utm_tr[2] < p_utm[2])
        utm_tr[2] = p_utm[2];

      if (utm_bl[0] > p_utm[0])
        utm_bl[0] = p_utm[0];
      if (utm_bl[1] > p_utm[1])
        utm_bl[1] = p_utm[1];
      if (utm_bl[2] > p_utm[2])
        utm_bl[2] = p_utm[2];
      nr_of_points++;
    }
  }
  utm_meridian_convergence_ = gamma * M_PI / 180.0;
  RCLCPP_INFO(this->get_logger(), "gamma: %f Deg, utm_meridian_convergence: %f rad, scale: %f", gamma, utm_meridian_convergence_, k);

  /// adding border to outer points
  utm_bl = utm_bl + cv::Vec3d(-map_border_, -map_border_, 0.);
  utm_tr = utm_tr + cv::Vec3d(+map_border_, +map_border_, 0.);

  std::shared_ptr<ObjectMap> object_map = std::make_shared<ObjectMap>();

  /// compute map size
  int cols = fabs((utm_tr[0] - utm_bl[0])) / resolution_;
  int rows = fabs((utm_tr[1] - utm_bl[1])) / resolution_;
  cols += cols % 2;
  rows += rows % 2;
  object_map->mat() = cv::Mat(rows, cols, CV_8U, cv::Scalar(ObjectMap::Cell::CELL_UNKNOWN));

  cv::Point2d bottom_left(0, -rows * resolution_);
  object_map->init(object_map->mat().size(), resolution_, bottom_left, utm_bl + cv::Vec3d(0, 0, utm_z_offset_), utm_zone, utm_northp);

  RCLCPP_INFO(this->get_logger(), "%s", object_map->info_map().c_str());
  RCLCPP_INFO(this->get_logger(), "%s", object_map->info_geo().c_str());

  /// computing map_points for all objects in map frame
  object_map->comptue_map_points(*msg_objects_tmp);
  msg_objects_tmp->header.frame_id = frame_map_;
  object_map->draw(*msg_objects_tmp);

  cv::Mat &img_src = object_map->mat();

  if (!debug_dest_folder_.empty())
  {
    cv::imwrite(debug_dest_folder_ + "objects.png", img_src);
  }

  auto occupancy_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
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

  if (show_map_)
    object_map_->imshow(100);

  msg_objects_processed_ = msg_objects_tmp;
  static bool file_written_processed = false;
  if (!file_written_processed && !debug_dest_folder_.empty())
  {
    file_written_processed = true;
    std::string json_file(debug_dest_folder_ + "objects_processed.json");
    RCLCPP_INFO(this->get_logger(), "writing debug json file to: %s", json_file.c_str());
    tuw_json::write(json_file, "objects", tuw_json::toJson(*msg_objects_processed_));
  }

  occupancy_map_processed_ = occupancy_map_;

  services_init_providors();

  publish_transforms_utm_map();
  publish_map();
  publish_marker();
}

void ObjectMapNode::publish_marker()
{
  static rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_utm;
  static std::shared_ptr<visualization_msgs::msg::Marker> marker_msg_utm;
  if (publish_marker_ && !msg_objects_processed_)
    return;
  if (!object_map_)
    return;
  if (!marker_msg_utm)
  {
    auto marker = std::make_shared<visualization_msgs::msg::Marker>();
    pub_marker_utm = this->create_publisher<visualization_msgs::msg::Marker>("utm/plants", 10);

    marker->header.frame_id = frame_utm_;
    marker->header.stamp = this->now();
    marker->id = 0;
    marker->ns = "objects";
    marker->type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker->action = visualization_msgs::msg::Marker::ADD;
    marker->pose.position.x = 0;
    marker->pose.position.y = 0;
    marker->pose.position.z = 0;
    marker->pose.orientation.x = 0.0;
    marker->pose.orientation.y = 0.0;
    marker->pose.orientation.z = 0.0;
    marker->pose.orientation.w = 1.0;
    marker->scale.x = 1.0;
    marker->scale.y = 1.0;
    marker->scale.z = 1.0;
    marker->color.r = 1.0;
    marker->color.g = 0.0;
    marker->color.b = 0.0;
    marker->color.a = 0.5;
    for (const auto &o : msg_objects_processed_->objects)
    {
      for (const auto &g : o.geo_points)
      {
        cv::Vec3d p_utm = object_map_->lla2utm(cv::Vec3d(g.latitude, g.longitude, g.altitude));
        geometry_msgs::msg::Point p;
        p.x = p_utm[0];
        p.y = p_utm[1];
        p.z = p_utm[2];
        marker->points.push_back(std::move(p));
      }
    }
    marker_msg_utm = marker;
  }
  pub_marker_utm->publish(*marker_msg_utm);

  static rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_map;
  static std::shared_ptr<visualization_msgs::msg::Marker> marker_msg_map; /// marker msgs computed
  if (!marker_msg_map)
  {
    auto marker = std::make_shared<visualization_msgs::msg::Marker>();
    pub_marker_map = this->create_publisher<visualization_msgs::msg::Marker>("map/plants", 10);
    marker->header.frame_id = frame_map_;
    marker->header.stamp = this->now();
    marker->id = 0;
    marker->ns = "objects";
    marker->type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker->action = visualization_msgs::msg::Marker::ADD;
    marker->pose.position.x = 0;
    marker->pose.position.y = 0;
    marker->pose.position.z = 0;
    marker->pose.orientation.x = 0.0;
    marker->pose.orientation.y = 0.0;
    marker->pose.orientation.z = 0.0;
    marker->pose.orientation.w = 1.0;

    marker->scale.x = 0.1;
    marker->scale.y = 0.1;
    marker->scale.z = 0.1;
    marker->color.r = 0.0;
    marker->color.g = 1.0;
    marker->color.b = 0.0;
    marker->color.a = 0.5;
    for (const auto &o : msg_objects_processed_->objects)
    {
      for (const auto &g : o.geo_points)
      {
        cv::Vec3d p_map = object_map_->lla2world(cv::Vec3d(g.latitude, g.longitude, g.altitude));
        geometry_msgs::msg::Point p;
        p.x = p_map[0];
        p.y = p_map[1];
        p.z = p_map[2];
        marker->points.push_back(std::move(p));
      }
    }
    marker_msg_map = marker;
  }
  pub_marker_map->publish(*marker_msg_map);
}

void ObjectMapNode::publish_transforms_utm_map()
{
  RCLCPP_INFO(this->get_logger(), "publish_transforms_utm_map");

  if (publish_tf_ && object_map_)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = frame_utm_;
    tf.child_frame_id = frame_map_;
    cv::Vec3d utm_bl = object_map_->utm();
    tf.transform.translation.x = utm_bl[0];
    tf.transform.translation.y = utm_bl[1];
    tf.transform.translation.z = utm_bl[2];
    RCLCPP_INFO_ONCE(this->get_logger(), "publish TF: frame_id: %s, child_frame_id: %s", tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
    static bool file_written = false;
    if (!file_written && !debug_dest_folder_.empty())
    {
      /// write a human readable file with the transform for debugging reasons
      file_written = true;
      std::string yaml_file(debug_dest_folder_ + "transform.txt");
      std::ofstream yaml_datei(yaml_file);
      RCLCPP_INFO(this->get_logger(), "writing debug yaml file to: %s", yaml_file.c_str());
      if (yaml_datei.is_open())
      {
        char cmd[0x1FF];
        sprintf(cmd, "ros2 run tf2_ros static_transform_publisher %lf %lf %lf 0.0 0.0 0.0 1.0 %s %s",
                utm_bl[0], utm_bl[1], utm_bl[2], tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
        yaml_datei << "# objects location " << std::endl;
        yaml_datei << "# utm zone " << object_map_->zone();
        yaml_datei << (object_map_->is_north() ? "north" : "south") << std::endl;
        yaml_datei << cmd << std::endl;
        yaml_datei.close();
      }
      std::string object_map_param_file(debug_dest_folder_ + "mapimage.jgw");
      tuw::WorldFile wf;
      wf.resolution_x = object_map_->resolution_x();
      wf.resolution_y = object_map_->resolution_y();
      wf.coordinate_x = object_map_->utm()[0];
      wf.coordinate_y = object_map_->utm()[1];
      wf.coordinate_z = object_map_->utm()[2];
      wf.origin_x = object_map_->origin_x();
      wf.origin_y = object_map_->origin_y();
      wf.write_jgw(object_map_param_file);
    }
    broadcaster_utm_->sendTransform(tf);
  }
}
void ObjectMapNode::publish_transforms_top_left()
{

  RCLCPP_DEBUG(this->get_logger(), "publish_transforms_top_left");
  static std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  if (publish_tf_ && object_map_)
  {
    if (!tf_broadcaster)
      tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = frame_utm_;
    tf.child_frame_id = frame_map_ + std::string("_top_left");
    cv::Vec3d utm_bl = object_map_->utm();
    tf.transform.translation.x = utm_bl[0];
    tf.transform.translation.y = utm_bl[1];
    tf.transform.translation.z = utm_bl[2];
    RCLCPP_INFO_ONCE(this->get_logger(), "publish TF: frame_id: %s, child_frame_id: %s", tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
    tf_broadcaster->sendTransform(tf);
  }
}


void ObjectMapNode::publish_map()
{
  static unsigned long publish_count = 0;
  if (occupancy_map_processed_)
  {
    pub_occupancy_grid_map_->publish(*occupancy_map_processed_);
    if (show_map_)
      object_map_->imshow(1000);
    pub_objects_on_map_->publish(*msg_objects_processed_);
    RCLCPP_INFO(this->get_logger(), "published occupancy_map %4ld", publish_count++);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "callback_timer no occupancy_map computed");
  }
}

void ObjectMapNode::callback_get_occ_map(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<nav_msgs::srv::GetMap::Request> /*request*/,
    std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
{
  RCLCPP_INFO(get_logger(), "Handling GetMap request");
  if (occupancy_map_processed_)
    response->map = *occupancy_map_processed_;
  else
    RCLCPP_WARN(get_logger(), "map not ready");
}

void ObjectMapNode::callback_get_objects(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<tuw_object_map_msgs::srv::GetObjects::Request> /*request*/,
    std::shared_ptr<tuw_object_map_msgs::srv::GetObjects::Response> response)
{
  RCLCPP_INFO(get_logger(), "Handling GetMap request");
  if (msg_objects_processed_)
    response->map = *msg_objects_processed_;
}

void ObjectMapNode::declare_parameters()
{
  declare_parameters_with_description("pub_interval", 5, "Publishing interval in seconds. If 0 or less, the graph is published once.", 0, 100, 1);
  declare_parameters_with_description("timeout_service_call", 5, "Timeout on the GetGraph servide after startup in seconds. If 0, the timeout will be infinity", 0, 600, 1);
  declare_parameters_with_description("debug_folder", "/tmp/ros", "Debug root folder, if set it information is stored in a subfolder of debug_folder with the node name.");
  declare_parameters_with_description("publish_tf", true, "On true a tf from frame_utm to frame_map is published");
  declare_parameters_with_description("publish_marker", true, "On true objects are published using marker msgs");
  declare_parameters_with_description("frame_map", "map", "Name of the map frame, only need if publish_tf == true");
  declare_parameters_with_description("frame_utm", "utm", "Name of the utm frame, only need if publish_tf == true");
  declare_parameters_with_description("resolution", 0.1, "Resolution of the generated map [m/pix]");
  declare_parameters_with_description("map_border", 10.1, "Border on the created map [meter]");
  declare_parameters_with_description("show_map", false, "Shows the map in a opencv window");
  declare_parameters_with_description("utm_z_offset", 0.0, "offest on Z for the utm -> map frame");
}

bool ObjectMapNode::read_dynamic_parameters()
{

  static bool first_call = true; /// varible to identify the first time the fnc was called to init all variables
  bool changes = false;          /// used to identify changes

  update_parameter_and_log("resolution", resolution_, changes, first_call);
  update_parameter_and_log("map_border", map_border_, changes, first_call);
  update_parameter_and_log("publish_marker", publish_marker_, changes, first_call);
  update_parameter_and_log("show_map", show_map_, changes, first_call);
  update_parameter_and_log("publish_tf", publish_tf_, changes, first_call);
  update_parameter_and_log("utm_z_offset", utm_z_offset_, changes, first_call);

  first_call = false;
  return changes;
}

void ObjectMapNode::read_static_parameters()
{
  get_parameter_and_log("debug_folder", debug_root_folder_);
  get_parameter_and_log("pub_interval", pub_interval_);
  get_parameter_and_log("timeout_service_call", timeout_service_call_);
  get_parameter_and_log("frame_utm", frame_utm_);
  get_parameter_and_log("frame_map", frame_map_);
}
