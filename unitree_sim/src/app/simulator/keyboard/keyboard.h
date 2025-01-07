#ifndef UNITREE_SIM_SRC_APP_SIMULATOR_KEYBOARD_KEYBOARD_H_
#define UNITREE_SIM_SRC_APP_SIMULATOR_KEYBOARD_KEYBOARD_H_

#include <cstdint>

#define CUSTOMKEY_CTRL_R 82
#define CUSTOMKEY_CTRL_T 84
#define CUSTOMKEY_CTRL_Y 89
#define CUSTOMKEY_CTRL_F 70
#define CUSTOMKEY_CTRL_G 71
#define CUSTOMKEY_CTRL_H 72
#define CUSTOMKEY_CTRL_V 86
#define CUSTOMKEY_CTRL_B 66
#define CUSTOMKEY_CTRL_N 78
#define CUSTOMKEY_CTRL_Z 90
#define CUSTOMKEY_CTRL_X 88
#define CUSTOMKEY_CTRL_C 67
#define CUSTOMKEY_CTRL_DOT 46

struct Keyboard {
  double lx, ly;
  double rx, ry;
  uint16_t keys;
  bool pressed;

  void clear() {
    lx = ly = rx = ry = 0;
    keys = 0;
    pressed = false;
    return;
  }
};

#endif  // UNITREE_SIM_SRC_APP_SIMULATOR_KEYBOARD_KEYBOARD_H_
