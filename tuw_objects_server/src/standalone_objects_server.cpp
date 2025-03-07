#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tuw_objects_server/objects_server_node.hpp"

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  std::shared_ptr<tuw_objects::ObjectsServerNode> node =
    std::make_shared<tuw_objects::ObjectsServerNode>("objects_server");
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
