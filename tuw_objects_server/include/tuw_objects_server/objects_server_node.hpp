#ifndef TUW_OBJECT_MAP_SERVER__TUW_OBJECTS_SERVER_NODE_HPP_
#define TUW_OBJECT_MAP_SERVER__TUW_OBJECTS_SERVER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <tuw_object_map_msgs/msg/object_map.hpp>
#include <tuw_object_map_msgs/srv/get_object_map.hpp>

namespace tuw_objects
{
  class ObjectsServerNode : public rclcpp::Node
  {
  public:
    ObjectsServerNode(const std::string &node_name);

  private:
    const std::string topic_name_objects_{"objects"};       /// topic name to subscribe for the map topic
    const std::string service_name_objects_{"get_objects"}; /// service name provided for GetObjectMap

    // Publisher for the object map
    rclcpp::Publisher<tuw_object_map_msgs::msg::ObjectMap>::SharedPtr pub_object_map_;

    // A service to provide the occupancy grid (GetMap) and the message to return
    rclcpp::Service<tuw_object_map_msgs::srv::GetObjectMap>::SharedPtr srv_map_;

    // object map read from file to be published
    tuw_object_map_msgs::msg::ObjectMap::SharedPtr object_map_;

    // thread for the time
    std::shared_ptr<std::thread> process_;

    // thread for the time
    rclcpp::TimerBase::SharedPtr timer_;

    // callback time
    void callback_timer();

    // read object json file into
    void read_object_map(const std::string &filename);

    // frame id use
    std::string frame_id_;

    // file read
    std::string object_map_json_;

    // file loop rate in seconds if zero node publish only onces
    int loop_rate_;

    /**
     * declares parameters
     */
    void declare_parameters();

    /**
     * read parameters and sets defaults
     */
    void read_parameters();

    /**
     * publish map
     */
    void publish_map();

    /**
     * @brief Map getting service callback
     * @param request_header Service request header
     * @param request Service request
     * @param response Service response
     */
    void callback_get_map(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<tuw_object_map_msgs::srv::GetObjectMap::Request> request,
        std::shared_ptr<tuw_object_map_msgs::srv::GetObjectMap::Response> response);
  };
}
#endif // TUW_OBJECT_MAP_SERVER__TUW_OBJECTS_SERVER_NODE_HPP_
