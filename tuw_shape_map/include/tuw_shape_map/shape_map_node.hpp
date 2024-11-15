#ifndef TUW_SHAPE_MAP__SHAPE_MAP_NODE_HPP_
#define TUW_SHAPE_MAP__SHAPE_MAP_NODE_HPP_

#include <memory>
#include <opencv2/core/core.hpp>
#include <tuw_ros2_utils/node.hpp>
#include <thread>
#include <tuw_shape_map/shape_map.hpp>
#include <tuw_object_msgs/msg/shape_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tuw_object_msgs/srv/get_shape_array.hpp>
#include <geographic_msgs/msg/geo_pose.hpp>

namespace tuw_shape_map
{
  class ObjectMapNode : public tuw::Node
  {
  public:
    ObjectMapNode(const std::string &node_name);

  private:
    std::mutex lock_;                                                         /// mutex to allow one process a time
    const std::string topic_name_objects_to_subscribe_{"shapes"};             /// topic name to subscribe
    const std::string service_name_objects_to_call_{"get_shapes"};            /// service name to call
    const std::string topic_name_map_to_provide_{"map"};                      /// topic name to provide
    const std::string topic_name_geo_pose_map_{"geo_pose_map"};               /// topic name to provide
    const std::string topic_name_objects_to_provide_{"shapes_on_map"};        /// topic name to provide objects with computed map_points
    const std::string service_name_map_to_provide_{"get_map"};                /// service name to provide
    const std::string service_name_objects_to_provide_{"get_objects_on_map"}; /// service name to provide objects with computed map_points

    /// subscriber an object map
    rclcpp::Subscription<tuw_object_msgs::msg::ShapeArray>::ConstSharedPtr sub_object_map_;

    /// callback for the incomming object map
    void callback_object_map(const tuw_object_msgs::msg::ShapeArray::SharedPtr msg);

    /// publisher for objects on map
    rclcpp::Publisher<tuw_object_msgs::msg::ShapeArray>::SharedPtr pub_objects_on_map_;

    // last received objects
    tuw_object_msgs::msg::ShapeArray::SharedPtr msg_objects_received_;

    // last processed objects
    tuw_object_msgs::msg::ShapeArray::SharedPtr msg_objects_processed_;

    // timer for loop_rate
    rclcpp::TimerBase::SharedPtr timer_;
    
    // callbacks
    void on_timer();

    /// Object map objecet need to compute the occupancy grid as well as marker and transforms
    double utm_meridian_convergence_;

    int pub_interval_;              /// static parameter: republishing interval
    int timeout_service_call_;      /// static parameter: how long should the node try to call the GetGraph servide after startup
    std::string frame_map_;         /// static parameter: Name of the map frame, only need if publish_tf == true
    std::string frame_utm_;         /// static parameter: Name of the utm frame, only need if publish_tf == true
    std::string debug_root_folder_; /// static parameter: folder name with debug information from tuw_ tools
    std::string debug_dest_folder_; /// generated parameter: based on debug_root_folder_ + node name
    bool publish_tf_;               /// dynamic parameter: on true a tf from frame_utm to frame_map is published
    bool publish_tf_rotation_;      /// dynamic parameter: On true it adds a rotiation to the tf published caused by the projection. Only in combinaltion with publish_tf.
    bool publish_marker_;           /// dynamic parameter: on true objects are published using marker msgs
    double map_border_;             /// dynamic parameter: Border on the created map [meter]
    double resolution_;             /// dynamic parameter: Resolution of the generated map [m/pix]
    bool show_map_;                 /// dynamic parameter: Shows the map in a opencv window
    double utm_z_offset_;           /// dynamic parameter: z offset on the location of the map

    /// starts the computation
    void process_objects(const tuw_object_msgs::msg::ShapeArray &objects);
    void declare_parameters();      // declare parameters
    void read_static_parameters();  // ready the static parameters
    bool read_dynamic_parameters(); // ready the dynamic parameters and returns true on changes
  };
}
#endif // TUW_SHAPE_MAP__SHAPE_MAP_NODE_HPP_
