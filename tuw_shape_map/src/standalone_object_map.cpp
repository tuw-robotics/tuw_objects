#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tuw_shape_map/object_map_node.hpp"

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  std::shared_ptr<tuw_object_map::ObjectMapNode> node =
    std::make_shared<tuw_object_map::ObjectMapNode>("object_map");
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
