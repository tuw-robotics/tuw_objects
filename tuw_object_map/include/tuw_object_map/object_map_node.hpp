#ifndef TUW_OBJECT_MAP__OBJECT_MAP_NODE_HPP_
#define TUW_OBJECT_MAP__OBJECT_MAP_NODE_HPP_

#include <memory>
#include <opencv2/core/core.hpp>
#include <tuw/node.hpp>
#include <thread>
#include <tuw_object_map/object_map.hpp>
#include <tuw_object_map_msgs/msg/objects.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tuw_object_map_msgs/srv/get_objects.hpp>
#include <geographic_msgs/msg/geo_pose.hpp>

namespace tuw_object_map
{
  class ObjectMapNode : public tuw::Node
  {
  public:
    ObjectMapNode(const std::string &node_name);

  private:
    std::mutex lock_;                                                         /// mutex to allow one process a time
    const std::string topic_name_objects_to_subscribe_{"objects"};            /// topic name to subscribe
    const std::string service_name_objects_to_call_{"get_objects"};           /// service name to call
    const std::string topic_name_map_to_provide_{"map"};                      /// topic name to provide
    const std::string topic_name_geo_pose_map_{"geo_pose_map"};               /// topic name to provide
    const std::string topic_name_objects_to_provide_{"objects_on_map"};       /// topic name to provide objects with computed map_points
    const std::string service_name_map_to_provide_{"get_map"};                /// service name to provide
    const std::string service_name_objects_to_provide_{"get_objects_on_map"}; /// service name to provide objects with computed map_points

    /// subscriber an object map
    rclcpp::Subscription<tuw_object_map_msgs::msg::Objects>::ConstSharedPtr sub_object_map_;

    /// callback for the incomming object map
    void callback_object_map(const tuw_object_map_msgs::msg::Objects::SharedPtr msg);

    /// publisher for the computed map (OccupancyGrid)
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_grid_map_;

    /// publisher map pose
    rclcpp::Publisher<geographic_msgs::msg::GeoPose>::SharedPtr pub_geo_pose_map_;

    /// publisher for objects on map
    rclcpp::Publisher<tuw_object_map_msgs::msg::Objects>::SharedPtr pub_objects_on_map_;

    // last received graph
    tuw_object_map_msgs::msg::Objects::SharedPtr msg_objects_received_;

    // last processed graph
    tuw_object_map_msgs::msg::Objects::SharedPtr msg_objects_processed_;
    void service_objects_reqeust();

    // initialize services
    void services_init_providors();

    // computed map orition location
    geographic_msgs::msg::GeoPose::SharedPtr geo_pose_map_;

    // last processed OccupancyGrid
    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map_processed_;

    // A service to provide the occupancy grid (GetMap) and the message to return
    rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr srv_occ_map_;

    /**
     * @brief service callback for an OccupancyGrid Map
     * @param request_header Service request header
     * @param request Service request
     * @param response Service response
     */
    void callback_get_occ_map(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
        std::shared_ptr<nav_msgs::srv::GetMap::Response> response);

    // A service to provide the object map (GetObjectMap)
    rclcpp::Service<tuw_object_map_msgs::srv::GetObjects>::SharedPtr srv_object_map_;

    /**
     * @brief Objects service callback
     * @param request_header Service request header
     * @param request Service request
     * @param response Service response
     */
    void callback_get_objects(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<tuw_object_map_msgs::srv::GetObjects::Request> request,
        std::shared_ptr<tuw_object_map_msgs::srv::GetObjects::Response> response);

    rclcpp::TimerBase::SharedPtr timer_;           /// timer for loop_rate

    // Used for publishing the static utm->map
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_utm_;

    // callbacks
    void on_timer();

    /// Object map objecet need to compute the occupancy grid as well as marker and transforms
    std::shared_ptr<ObjectMap> object_map_;
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
    void process_objects(const tuw_object_map_msgs::msg::Objects &objects);
    void declare_parameters();      // declare parameters
    void read_static_parameters();  // ready the static parameters
    bool read_dynamic_parameters(); // ready the dynamic parameters and returns true on changes
    void publish_transforms_utm_map();
    void publish_transforms_top_left();
    void publish_marker();
    void publish_map();
  };
}
#endif // TUW_OBJECT_MAP__OBJECT_MAP_NODE_HPP_
