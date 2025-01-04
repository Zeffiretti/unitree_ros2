#include "app/controller/lowlevel_controller.h"

#include <string>

// #include <common/motor_crc.h>

// #include "app/controller/motor_crc.h"

namespace unitreesim::ros2 {
LowlevelControler::LowlevelControler(const std::string& node_name, int frequency) : rclcpp::Node(node_name) {
  // the cmd_puber is set to subscribe "/lowcmd" topic
  cmd_puber = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

  // Initialize lowcmd
  InitCmd();

  // Running time count
  time_consumed = 0.0;
  dt = 1.0 / frequency;

  low_state_suber = this->create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10, std::bind(&LowlevelControler::LowStateCallback, this, std::placeholders::_1));
  int ms = 1000 / frequency;
  timer_ = this->create_wall_timer(std::chrono::milliseconds(ms), std::bind(&LowlevelControler::TimerCallback, this));
}

LowlevelControler::~LowlevelControler() {}

void LowlevelControler::InitCmd() {
  cmd_msg.head[0] = 0xFE;
  cmd_msg.head[1] = 0xEF;
  cmd_msg.level_flag = 0xFF;
  cmd_msg.gpio = 0;

  for (int i = 0; i < 20; i++) {
    // cmd_msg.motor_cmd.
    cmd_msg.motor_cmd[i].mode = 0x01;  // Set toque mode, 0x00 is passive mode
    cmd_msg.motor_cmd[i].q = PosStopF;
    cmd_msg.motor_cmd[i].kp = kp;
    cmd_msg.motor_cmd[i].dq = VelStopF;
    cmd_msg.motor_cmd[i].kd = kd;
    cmd_msg.motor_cmd[i].tau = 0;
  }
}

void LowlevelControler::TimerCallback() {
  if (wait_for_start_pos) {
    RCLCPP_INFO(this->get_logger(), "First run");
    // first_run = false;
    return;
  }

  if (done) {
    RCLCPP_INFO(this->get_logger(), "Done");
    exit(0);
  }

  auto current_time = std::chrono::high_resolution_clock::now();
  time_consumed = std::chrono::duration<float>(current_time - start_time).count();

  float start_pos[12] = {0};
  float target_pos[12] = {0};
  float phase = CalcPhase(time_consumed, start_pos, target_pos);
  // RCLCPP_INFO(this->get_logger(), "Phase: %f", phase);
  // RCLCPP_INFO(this->get_logger(), "Start: %f %f %f %f %f %f %f %f %f %f %f %f", start_pos[0], start_pos[1],
  //             start_pos[2], start_pos[3], start_pos[4], start_pos[5], start_pos[6], start_pos[7], start_pos[8],
  //             start_pos[9], start_pos[10], start_pos[11]);
  // RCLCPP_INFO(this->get_logger(), "Target: %f %f %f %f %f %f %f %f %f %f %f %f", target_pos[0], target_pos[1],
  //             target_pos[2], target_pos[3], target_pos[4], target_pos[5], target_pos[6], target_pos[7],
  //             target_pos[8], target_pos[9], target_pos[10], target_pos[11]);

  // start_pos

  for (int i = 0; i < 12; i++) {
    cmd_msg.motor_cmd[i].q = start_pos[i] + phase * (target_pos[i] - start_pos[i]);
    cmd_msg.motor_cmd[i].kp = kp;
    cmd_msg.motor_cmd[i].dq = 0;
    cmd_msg.motor_cmd[i].kd = kd;
    cmd_msg.motor_cmd[i].tau = 0;
  }

  // RCLCPP_INFO(this->get_logger(), " %f %f %f %f %f %f %f %f %f %f %f %f", cmd_msg.motor_cmd[0].q,
  //             cmd_msg.motor_cmd[1].q, cmd_msg.motor_cmd[2].q, cmd_msg.motor_cmd[3].q, cmd_msg.motor_cmd[4].q,
  //             cmd_msg.motor_cmd[5].q, cmd_msg.motor_cmd[6].q, cmd_msg.motor_cmd[7].q, cmd_msg.motor_cmd[8].q,
  //             cmd_msg.motor_cmd[9].q, cmd_msg.motor_cmd[10].q, cmd_msg.motor_cmd[11].q);
  get_crc(cmd_msg);
  cmd_puber->publish(cmd_msg);
  // RCLCPP_INFO(this->get_logger(), "Published low cmd");
}

void LowlevelControler::LowStateCallback(const unitree_go::msg::LowState::SharedPtr msg) {
  if (wait_for_start_pos) {
    for (int i = 0; i < 12; i++) {
      _startPos[i] = msg->motor_state[i].q;
    }
    RCLCPP_INFO(this->get_logger(), "===============================================================");
    RCLCPP_INFO(this->get_logger(), "Start position received: ");
    for (int i = 0; i < 12; i++) {
      RCLCPP_INFO(this->get_logger(), "\tMotor %d: %f", i, _startPos[i]);
    }
    RCLCPP_INFO(this->get_logger(), "===============================================================");
    time_consumed = 0.0;
    wait_for_start_pos = false;
    start_time = std::chrono::high_resolution_clock::now();
  }
}

float LowlevelControler::CalcPhase(float current_time, float* start_pos, float* target_pos) {
  if (wait_for_start_pos) {
    return 0;
  }

  if (current_time < _duration_1) {
    std::copy(std::begin(_startPos), std::end(_startPos), start_pos);
    std::copy(std::begin(_targetPos_1), std::end(_targetPos_1), target_pos);
    return current_time / _duration_1;
  } else if (current_time < _duration_1 + _duration_2) {
    std::copy(std::begin(_targetPos_1), std::end(_targetPos_1), start_pos);
    std::copy(std::begin(_targetPos_2), std::end(_targetPos_2), target_pos);
    return (current_time - _duration_1) / _duration_2;
  } else if (current_time < _duration_1 + _duration_2 + _duration_3) {
    std::copy(std::begin(_targetPos_2), std::end(_targetPos_2), start_pos);
    std::copy(std::begin(_targetPos_2), std::end(_targetPos_2), target_pos);
    return (current_time - _duration_1 - _duration_2) / _duration_3;
  } else if (current_time < _duration_1 + _duration_2 + _duration_3 + _duration_4) {
    std::copy(std::begin(_targetPos_2), std::end(_targetPos_3), start_pos);
    std::copy(std::begin(_targetPos_3), std::end(_targetPos_3), target_pos);
    return (current_time - _duration_1 - _duration_2 - _duration_3) / _duration_4;
  } else {
    std::copy(std::begin(_targetPos_3), std::end(_targetPos_3), start_pos);
    std::copy(std::begin(_targetPos_3), std::end(_targetPos_3), target_pos);
    done = true;
    return 1;
  }
}
}  // namespace unitreesim::ros2
