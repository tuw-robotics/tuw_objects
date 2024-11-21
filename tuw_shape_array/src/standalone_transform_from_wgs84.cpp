#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tuw_shape_array/transform_from_wgs84_node.hpp"

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  std::shared_ptr<tuw_shape_array::FromWGS84Node> node =
    std::make_shared<tuw_shape_array::FromWGS84Node>("from_gws84");
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
