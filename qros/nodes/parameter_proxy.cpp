#include "parameter_proxy_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParameterProxyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
