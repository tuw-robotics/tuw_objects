#ifndef TUW_SHAPE_MAP__SHAPE_MAP_NODE_HPP_
#define TUW_SHAPE_MAP__SHAPE_MAP_NODE_HPP_

#include <memory>
#include <thread>
#include <tf2/LinearMath/Transform.h>
#include <tuw_ros2_utils/node.hpp>
#include <tuw_object_msgs/shape.hpp>
#include <tuw_object_msgs/msg/shape_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace tuw_shape_map
{
  class ObjectMapNode : public tuw::Node
  {
  public:
    ObjectMapNode(const std::string &node_name);

  private:
    const std::string topic_name_shaoes_to_subscribe_{"shapes"};                 /// topic name to subscribe
    const std::string service_name_shapes_to_call_{"get_shapes"};                /// service name to call
    const std::string topic_name_map_to_provide_{"map"};                         /// topic name to provide
    const std::string topic_name_geo_pose_map_{"geo_pose_map"};                  /// topic name to provide
    const std::string topic_name_shapes_to_provide_{"shapes_transformed"};       /// topic name to provide objects with computed map_points
    const std::string service_name_map_to_provide_{"get_map"};                   /// service name to provide
    const std::string service_name_shapes_to_provide_{"get_shapes_transformed"}; /// service name to provide objects with computed map_points

    /// subscriber incomming shapes
    rclcpp::Subscription<tuw_object_msgs::msg::ShapeArray>::ConstSharedPtr sub_shapes_;

    /// callback for the incomming shapes
    void callback_shapes(const tuw_object_msgs::msg::ShapeArray::SharedPtr msg);

    /// publisher for shapes transformed
    rclcpp::Publisher<tuw_object_msgs::msg::ShapeArray>::SharedPtr pub_shapes_transformed_;

    // last received shapes
    tuw_object_msgs::msg::ShapeArray::SharedPtr msg_shapes_received_;

    // msgs transformed into utm
    tuw_object_msgs::msg::ShapeArray::SharedPtr msg_shapes_utm_;

    // msgs transformed into map frame
    tuw_object_msgs::msg::ShapeArray::SharedPtr msg_shapes_map_;

    // last processed shapes
    tuw_object_msgs::msg::ShapeArray::SharedPtr msg_shapes_processed_;

    // Used for publishing the static utm->map
    //std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_utm_;

    std::shared_ptr<tuw_object_msgs::Shape> map_shape_;

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
    bool publish_marker_;           /// dynamic parameter: on true shapes are published using marker msgs
    double map_border_;             /// dynamic parameter: Border on the created map [meter]
    double resolution_;             /// dynamic parameter: Resolution of the generated map [m/pix]
    bool show_map_;                 /// dynamic parameter: Shows the map in a opencv window
    double utm_z_offset_;           /// dynamic parameter: z offset on the location of the map

    /// starts the computation
    void start_process(const tuw_object_msgs::msg::ShapeArray::SharedPtr msg);
    void transform_wgs84_to_utm(tuw_object_msgs::msg::ShapeArray::SharedPtr shapes, int utm_zone);
    void transform_utm_to_map(tuw_object_msgs::msg::ShapeArray::SharedPtr shapes, const tf2::Transform &tf);
    void compute_map_frame(tuw_object_msgs::msg::ShapeArray::SharedPtr shapes, double border, tf2::Transform &tf);

    void declare_parameters();      // declare parameters
    void read_static_parameters();  // ready the static parameters
    bool read_dynamic_parameters(); // ready the dynamic parameters and returns true on changes
    void publish_transforms_utm_map();
  };
}
#endif // TUW_SHAPE_MAP__SHAPE_MAP_NODE_HPP_
