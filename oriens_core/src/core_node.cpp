#include <rclcpp/rclcpp.hpp>
#include "oriens_core/core.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Core>("core");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}