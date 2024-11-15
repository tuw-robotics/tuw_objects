#include <filesystem>
#include <json/json.h>
#include <tuw_json/json.hpp>
#include <tuw_object_msgs/shape_array_json.hpp>
#include "tuw_shape_array/transform_shapes_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using namespace tuw_shape_map;

ObjectMapNode::ObjectMapNode(const std::string &node_name)
    : Node(node_name)
{
  declare_parameters();
  read_static_parameters();
  read_dynamic_parameters();

  if (!debug_root_folder_.empty())
  {
    if (debug_root_folder_.back() != '/')
      debug_root_folder_ += '/';
    debug_dest_folder_ = debug_root_folder_ + get_name() + "/";
    if (!std::filesystem::is_directory(debug_dest_folder_) || !std::filesystem::exists(debug_dest_folder_))
      std::filesystem::create_directories(debug_dest_folder_); // create debug destination folder for this node
    RCLCPP_INFO(this->get_logger(), "debug_dest_folder: %s", debug_dest_folder_.c_str());
  }

  pub_objects_on_map_ = this->create_publisher<tuw_object_msgs::msg::ShapeArray>(topic_name_objects_to_provide_, 10);
  sub_object_map_ = create_subscription<tuw_object_msgs::msg::ShapeArray>(topic_name_objects_to_subscribe_, 10, std::bind(&ObjectMapNode::callback_object_map, this, _1));

  if (pub_interval_ > 0)
  {
    timer_ = create_wall_timer(std::chrono::milliseconds(1000) * pub_interval_, std::bind(&ObjectMapNode::on_timer, this));
  }
}

void ObjectMapNode::on_timer()
{
  RCLCPP_INFO(this->get_logger(), "on_timer");
}

void ObjectMapNode::callback_object_map(
    const tuw_object_msgs::msg::ShapeArray::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "callback_objects");
  process_objects(*msg);
}

void ObjectMapNode::process_objects(const tuw_object_msgs::msg::ShapeArray &msg)
{

}


void ObjectMapNode::declare_parameters()
{
  declare_parameters_with_description("pub_interval", 5, "Publishing interval in seconds. If 0 or less, the graph is published once.", 0, 100, 1);
  declare_parameters_with_description("timeout_service_call", 5, "Timeout on the GetGraph servide after startup in seconds. If 0, the timeout will be infinity", 0, 600, 1);
  declare_parameters_with_description("debug_folder", "/tmp/ros", "Debug root folder, if set it information is stored in a subfolder of debug_folder with the node name.");
  declare_parameters_with_description("publish_tf", true, "On true a tf from frame_utm to frame_map is published");
  declare_parameters_with_description("publish_tf_rotation", false, "On true it adds a rotiation to the tf published caused by the projection. Only in combinaltion with publish_tf. ");
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
  update_parameter_and_log("publish_tf_rotation", publish_tf_rotation_, changes, first_call);
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
