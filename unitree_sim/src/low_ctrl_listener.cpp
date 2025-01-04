#include <rclcpp/rclcpp.hpp>

#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#define INFO_FOOT_STATE \
  1  // Set 1 to info foot states (foot position and velocity in body frame)
#define HIGH_FREQ \
  0  // Set 1 to subscribe to motion states with high frequencies (500Hz)

using std::placeholders::_1;

class motion_cmd_suber : public rclcpp::Node {
 public:
  motion_cmd_suber() : Node("motion_cmd_suber") {
    // the cmd_puber is set to subscribe "sportmodestate" or "lf/sportmodestate"
    // (low frequencies) topic
    auto topic_name = "/lowcmd";
    // if (HIGH_FREQ) {
    //   topic_name = "sportmodestate";
    // }

    // The suber  callback function is bind to
    // motion_state_suber::topic_callback
    suber = this->create_subscription<unitree_go::msg::LowCmd>(
        topic_name, 10, std::bind(&motion_cmd_suber::topic_callback, this, _1));
  }

 private:
  void topic_callback(unitree_go::msg::LowCmd::SharedPtr data) {
    RCLCPP_INFO(this->get_logger(), "Received low cmd: %f",
                data->motor_cmd[0].q);
  }

  // Create the suber to receive motion states of robot
  rclcpp::Subscription<unitree_go::msg::LowCmd>::SharedPtr suber;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  rclcpp::spin(
      std::make_shared<motion_cmd_suber>());  // Run ROS2 node which is make
                                              // share with motion_state_suber
                                              // class
  rclcpp::shutdown();
  return 0;
}
