#ifndef TUW_OBJECT_MAP__OBJECT_MAP_NODE_HPP_
#define TUW_OBJECT_MAP__OBJECT_MAP_NODE_HPP_

#include <memory>
#include <opencv2/core/core.hpp>
#include <tuw_ros2_utils/node.hpp>
#include <thread>
#include <tuw_object_map/object_map.hpp>
#include <tuw_object_map_msgs/msg/object_map.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tuw_object_map_msgs/srv/load_map.hpp>
#include <nav_msgs/srv/get_map.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tuw_object_map_msgs/srv/get_object_map.hpp>

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
    const std::string topic_name_objects_to_provide_{"objects_on_map"};       /// topic name to provide
    const std::string service_name_map_to_provide_{"get_map"};                /// service name to provide
    const std::string service_name_objects_to_provide_{"get_objects_on_map"}; /// service name to provide

    /// subscriber an object map
    rclcpp::Subscription<tuw_object_map_msgs::msg::ObjectMap>::ConstSharedPtr sub_object_map_;

    /// callback for the incomming object map
    void callback_object_map(const tuw_object_map_msgs::msg::ObjectMap::SharedPtr msg);

    /// publisher for the computed map (OccupancyGrid)
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_grid_map_;

    /// publisher for the updated  object_map holding now map_points (ObjectMap)
    rclcpp::Publisher<tuw_object_map_msgs::msg::ObjectMap>::SharedPtr pub_objects_;

    // last received graph
    tuw_object_map_msgs::msg::ObjectMap::SharedPtr msg_objects_received_;

    // last processed graph
    tuw_object_map_msgs::msg::ObjectMap::SharedPtr msg_objects_processed_;
    void service_objects_reqeust();

    // initialize services
    void services_init_providors();

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
    rclcpp::Service<tuw_object_map_msgs::srv::GetObjectMap>::SharedPtr srv_object_map_;

    /**
     * @brief Objects service callback
     * @param request_header Service request header
     * @param request Service request
     * @param response Service response
     */
    void callback_get_object_map(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<tuw_object_map_msgs::srv::GetObjectMap::Request> request,
        std::shared_ptr<tuw_object_map_msgs::srv::GetObjectMap::Response> response);

    /// publisher for marker msgs
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    std::shared_ptr<visualization_msgs::msg::Marker> marker_msg_; /// marker msgs computed

    rclcpp::TimerBase::SharedPtr timer_;           /// timer for loop_rate
    rclcpp::TimerBase::SharedPtr timer_transform_; /// timer to publish transformatins
    // callbacks
    void on_timer();

    /// function to load object map from file for debuging
    void load_object_map(const std::string &filename);

    /// Object map objecet need to compute the occupancy grid as well as marker and transforms
    std::shared_ptr<ObjectMap> object_map_;

    int loop_rate_;                 /// static parameter: republishing rate, not recomputation
    int timeout_service_call_;      /// static parameter: how long should the node try to call the GetGraph servide after startup
    std::string frame_map_;         /// static parameter: Name of the map frame, only need if publish_tf == true
    std::string frame_utm_;         /// static parameter: Name of the utm frame, only need if publish_tf == true
    std::string json_file_;         /// static parameter: Filename to load the object map from a json file if not set the node will wait for a msg on the topic
    std::string debug_root_folder_; /// static parameter: folder name with debug information from tuw_ tools
    std::string debug_dest_folder_; /// generated parameter: based on debug_root_folder_ + node name
    bool publish_tf_;               /// dynamic parameter: on true a tf from frame_utm to frame_map is published
    bool publish_marker_;           /// dynamic parameter: on true objects are published using marker msgs
    double map_border_;             /// dynamic parameter: Border on the created map [meter]
    double resolution_;             /// dynamic parameter: Resolution of the generated map [m/pix]
    bool show_map_;                 /// dynamic parameter: Shows the map in a opencv window

    /// starts the computation
    void process_objects(const tuw_object_map_msgs::msg::ObjectMap &objects);
    void declare_parameters();      // declare parameters
    void read_static_parameters();  // ready the static parameters
    bool read_dynamic_parameters(); // ready the dynamic parameters and returns true on changes
    void publish_transforms();
    void publish_marker();
    void publish_map();
    void publish_objects();


  };
}
#endif // TUW_OBJECT_MAP__OBJECT_MAP_NODE_HPP_
