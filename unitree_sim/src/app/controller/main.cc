#include <iostream>
#include <memory>

#include "app/controller/lowlevel_controller.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  auto node = std::make_shared<unitreesim::ros2::LowlevelControler>("lowlevel_controller", 50);
  rclcpp::spin(node);  // Run ROS2 node which is make share with LowlevelControler class
  return 0;
}
