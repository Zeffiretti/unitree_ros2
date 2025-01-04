#ifndef UNITREE_SIM_SRC_APP_CONTROLLER_LOWLEVEL_CONTROLLER_H_
#define UNITREE_SIM_SRC_APP_CONTROLLER_LOWLEVEL_CONTROLLER_H_

#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/bms_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/motor_cmd.hpp>

#include "common/motor_crc.h"

// #define TOPIC_LOWCMD "/lowcmd"
// #define TOPIC_LOWSTATE "/lowstate"

// constexpr double PosStopF = (2.146E+9f);
// constexpr double VelStopF = (16000.0f);

namespace unitreesim::ros2 {
class LowlevelControler : public rclcpp::Node {
 public:
  explicit LowlevelControler(const std::string& node_name = "lowlevel_controller", int frequency = 50);
  ~LowlevelControler();

  void TimerCallback();
  void LowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);

 private:
  float kp = 40.0;
  float kd = 2.0;
  double time_consumed = 0.0;
  float dt = 1.0 / 50;

  // float _targetPos_1[12] = {0};
  // float _targetPos_2[12] = {0};
  // float _targetPos_3[12] = {0};
  float _targetPos_1[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};
  float _targetPos_2[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3};
  float _targetPos_3[12] = {-0.35, 1.36, -2.65, 0.35, 1.36, -2.65, -0.5, 1.36, -2.65, 0.5, 1.36, -2.65};

  float _startPos[12] = {0};
  float _duration_1 = 1.0;  // time to reach target position 1
  float _duration_2 = 1.0;  // time to reach target position 2
  float _duration_3 = 2.0;  // time for staying at target position 2
  float _duration_4 = 1.8;  // time to reach target position 3

  bool wait_for_start_pos = true;
  bool done = false;
  std::chrono::high_resolution_clock::time_point start_time;

  // ros publisher and subscriber
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_puber;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_suber;

  // ros message
  unitree_go::msg::LowCmd cmd_msg;

  void InitCmd();
  float CalcPhase(float current_time, float* start_pos, float* target_pos);
};
}  // namespace unitreesim::ros2

#endif  // UNITREE_SIM_SRC_APP_CONTROLLER_LOWLEVEL_CONTROLLER_H_
