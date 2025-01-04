#ifndef UNITREE_SIM_SRC_APP_SIMULATOR_UNITREE_ROS2_BRIDGE_UNITREE_ROS2_BRIDGE_H_
#define UNITREE_SIM_SRC_APP_SIMULATOR_UNITREE_ROS2_BRIDGE_UNITREE_ROS2_BRIDGE_H_

#include <mujoco/mujoco.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include <unitree_go/msg/wireless_controller.hpp>
#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/low_state.hpp>
// #include <unitree_hg/msg/sport_mode_state.hpp>

#include "app/simulator/joystick/joystick.h"

namespace unitreesim::ros2 {
#define TOPIC_LOWSTATE "/lowstate"
#define TOPIC_HIGHSTATE "/sportmodestate"
#define TOPIC_LOWCMD "/lowcmd"
#define TOPIC_WIRELESS_CONTROLLER "/wirelesscontroller"
#define MOTOR_SENSOR_NUM 3
#define NUM_MOTOR_IDL_GO 20
#define NUM_MOTOR_IDL_HG 35

typedef union {
  struct {
    uint8_t R1 : 1;
    uint8_t L1 : 1;
    uint8_t start : 1;
    uint8_t select : 1;
    uint8_t R2 : 1;
    uint8_t L2 : 1;
    uint8_t F1 : 1;
    uint8_t F2 : 1;
    uint8_t A : 1;
    uint8_t B : 1;
    uint8_t X : 1;
    uint8_t Y : 1;
    uint8_t up : 1;
    uint8_t right : 1;
    uint8_t down : 1;
    uint8_t left : 1;
  } components;
  uint16_t value;
} xKeySwitchUnion;

typedef struct {
  uint8_t head[2];
  xKeySwitchUnion btn;
  float lx;
  float rx;
  float ry;
  float L2;
  float ly;

  uint8_t idle[16];
} xRockerBtnDataStruct;

// Defaults to xbox gamepad
struct JoystickId {
  std::map<std::string, int> axis = {
      {"LX", 0},  // Left stick axis x
      {"LY", 1},  // Left stick axis y
      {"RX", 3},  // Right stick axis x
      {"RY", 4},  // Right stick axis y
      {"LT", 2},  // Left trigger
      {"RT", 5},  // Right trigger
      {"DX", 6},  // Directional pad x
      {"DY", 7},  // Directional pad y
  };

  std::map<std::string, int> button = {
      {"X", 2}, {"Y", 3}, {"B", 1}, {"A", 0}, {"LB", 4}, {"RB", 5}, {"SELECT", 6}, {"START", 7},
  };
};

class UnitreeRos2Bridge : public rclcpp::Node {
 public:
  UnitreeRos2Bridge(mjModel* model, mjData* data, const std::string& node_name = "mujoco_simulator",
                    bool dry_run = true);
  ~UnitreeRos2Bridge();

  void CheckSensor();
  void PrintSceneInformation();
  void SetupJoystick(std::string device, std::string js_type, int bits);

 private:
  void LowCmdGoHandler(const unitree_go::msg::LowCmd::SharedPtr msg);
  void LowCmdHgHandler(const unitree_hg::msg::LowCmd::SharedPtr msg);

  void PublishLowStateGo();
  void PublishLowStateHg();
  void PublishHighState();
  void PublishWirelessController();
  void Run();

  rclcpp::Subscription<unitree_go::msg::LowCmd>::SharedPtr low_cmd_go_suber_;
  rclcpp::Subscription<unitree_hg::msg::LowCmd>::SharedPtr low_cmd_hg_suber_;

  rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr low_state_go_puber_;
  rclcpp::Publisher<unitree_hg::msg::LowState>::SharedPtr low_state_hg_puber_;
  rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr high_state_puber_;
  rclcpp::Publisher<unitree_go::msg::WirelessController>::SharedPtr wireless_controller_puber_;

  rclcpp::TimerBase::SharedPtr lowStatePuberTimer_;
  rclcpp::TimerBase::SharedPtr HighStatePuberTimer_;
  rclcpp::TimerBase::SharedPtr WirelessControllerPuberTimer_;

  xKeySwitchUnion dds_keys_ = {};
  xRockerBtnDataStruct wireless_remote_ = {};

  JoystickId js_id_;
  Joystick* js_;
  int max_value_ = (1 << 15);  // 16 bits joystick

  mjData* mj_data_;
  mjModel* mj_model_;

  int num_motor_ = 0;
  int dim_motor_sensor_ = 0;

  int have_imu_ = false;
  int have_frame_sensor_ = false;
  int idl_type_ = 0;  // 0: unitree_go, 1: unitree_hg

  // Unitree Go Message
  unitree_go::msg::LowState low_state_go_;
  unitree_go::msg::SportModeState high_state_;
  unitree_go::msg::WirelessController wireless_controller_;
  unitree_go::msg::LowCmd low_cmd_go_;

  // Unitree Hg Message
  unitree_hg::msg::LowState low_state_hg_;
  unitree_hg::msg::LowCmd low_cmd_hg_;

 private:
  void GetWirelessRemote();
};
}  // namespace unitreesim::ros2

#endif  // UNITREE_SIM_SRC_APP_SIMULATOR_UNITREE_ROS2_BRIDGE_UNITREE_ROS2_BRIDGE_H_
