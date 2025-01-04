cmake_minimum_required(VERSION 3.16)
# project(unitree_controller)

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(SIM_DEPENDENCIES
#     pthread
#     mujoco::mujoco
#     glfw
#     yaml-cpp
#     # unitree_sdk2
# )


add_library(unitreesim_common SHARED
  # motor_crc_hg.cpp
  motor_crc.cpp
  ros2_sport_client.cpp
)
target_include_directories(unitreesim_common PRIVATE ${UNITREE_INCLUDE_DIRS})
# target_link_libraries(unitree_controller ${SIM_DEPENDENCIES})
ament_target_dependencies(unitreesim_common ${DEPENDENCY_LIST})

# SET(CMAKE_BUILD_TYPE Release)