#ifndef TUW_OBJECT_MAP_SERVER__SHAPE_SERVER_NODE_HPP_
#define TUW_OBJECT_MAP_SERVER__SHAPE_SERVER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <tuw_object_msgs/msg/shape_array.hpp>
#include <tuw_object_msgs/srv/get_shape_array.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace tuw_shape_array
{
  class ShapeServerNode : public rclcpp::Node
  {
  public:
    ShapeServerNode(const std::string &node_name);

  private:
    const std::string topic_name_{"shapes"};       /// topic name to subscribe for the map topic
    const std::string service_get_{"get"};  /// service name provided for GetShapeArray
    const std::string service_publish_{"publish"}; /// service name to trigger a republish

    // Publisher for the shapes
    rclcpp::Publisher<tuw_object_msgs::msg::ShapeArray>::SharedPtr pub_shapes_;

    // A service to provide the shapes
    rclcpp::Service<tuw_object_msgs::srv::GetShapeArray>::SharedPtr srv_shapes_;

    // A service to trigger a publish
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_publish_;

    // shapes read from file to be published
    tuw_object_msgs::msg::ShapeArray::SharedPtr shapes_;

    // thread for the time
    std::shared_ptr<std::thread> process_;

    // thread for the time
    rclcpp::TimerBase::SharedPtr timer_;

    // callback time
    void callback_timer();

    // read object json file into
    void read_shape_array(const std::string &filename);

    // frame id use
    std::string frame_id_;

    // file read
    std::string shapes_json_;

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
     * publish shapes
     */
    void publish_shapes();

    /**
     * @brief Map getting service callback
     * @param request_header Service request header
     * @param request Service request
     * @param response Service response
     */
    void callback_get_shapes(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<tuw_object_msgs::srv::GetShapeArray::Request> request,
        std::shared_ptr<tuw_object_msgs::srv::GetShapeArray::Response> response);

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
#endif // TUW_OBJECT_MAP_SERVER__SHAPE_SERVER_NODE_HPP_
