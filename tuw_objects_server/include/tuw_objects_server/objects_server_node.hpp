#ifndef TUW_OBJECT_MAP_SERVER__TUW_OBJECTS_SERVER_NODE_HPP_
#define TUW_OBJECT_MAP_SERVER__TUW_OBJECTS_SERVER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <tuw_object_map_msgs/msg/objects.hpp>
#include <tuw_object_map_msgs/srv/get_objects.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace tuw_objects
{
  class ObjectsServerNode : public rclcpp::Node
  {
  public:
    ObjectsServerNode(const std::string &node_name);

  private:
    const std::string topic_name_objects_{"objects"};       /// topic name to subscribe for the map topic
    const std::string service_name_objects_{"get_objects"}; /// service name provided for GetObjectMap
    const std::string service_name_publish_{"publish"};     /// service name to trigger a republish

    // Publisher for the objects
    rclcpp::Publisher<tuw_object_map_msgs::msg::Objects>::SharedPtr pub_objects_;

    // A service to provide the objects
    rclcpp::Service<tuw_object_map_msgs::srv::GetObjects>::SharedPtr srv_objects_;

    // A service to trigger a publish
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_publish_;

    // objects read from file to be published
    tuw_object_map_msgs::msg::Objects::SharedPtr objects_;

    // thread for the time
    std::shared_ptr<std::thread> process_;

    // thread for the time
    rclcpp::TimerBase::SharedPtr timer_;

    // callback time
    void callback_timer();

    // read object json file into
    void read_objects(const std::string &filename);

    // frame id use
    std::string frame_id_;

    // file read
    std::string objects_json_;

    // file publishing interval in seconds if zero node publish only onces
    int pub_interval_;

    /**
     * declares parameters
     */
    void declare_parameters();

    /**
     * read parameters and sets defaults
     */
    void read_parameters();

    /**
     * publish objects
     */
    void publish_objects();

    /**
     * @brief Map getting service callback
     * @param request_header Service request header
     * @param request Service request
     * @param response Service response
     */
    void callback_get_objects(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<tuw_object_map_msgs::srv::GetObjects::Request> request,
        std::shared_ptr<tuw_object_map_msgs::srv::GetObjects::Response> response);

    /**
     * @brief Map getting service callback
     * @param request_header Service request header
     * @param request Service request
     * @param response Service response
     */
    void callback_publish(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  };
}
#endif // TUW_OBJECT_MAP_SERVER__TUW_OBJECTS_SERVER_NODE_HPP_
