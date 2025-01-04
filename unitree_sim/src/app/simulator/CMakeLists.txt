cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

FILE (GLOB SIM_SRC
    joystick/joystick.cc
    mujoco/*.cc
    unitree_ros2_bridge/*.cc
    # unitree_sdk2_bridge/*.cc
)

set(SIM_DEPENDENCIES
    pthread
    mujoco::mujoco
    glfw
    yaml-cpp
    # unitree_sdk2
)


add_executable(unitree_mujoco ${SIM_SRC} main.cc)
target_include_directories(unitree_mujoco PRIVATE ${UNITREE_INCLUDE_DIRS})
target_link_libraries(unitree_mujoco  ${SIM_DEPENDENCIES})
ament_target_dependencies(unitree_mujoco ${DEPENDENCY_LIST})

# add_executable(test test/test_unitree_sdk2.cpp)
# target_include_directories(test PRIVATE ${UNITREE_INCLUDE_DIRS})
# target_link_libraries(test unitree_sdk2)

add_executable(jstest joystick/jstest.cc joystick/joystick.cc)
target_include_directories(jstest PRIVATE ${UNITREE_INCLUDE_DIRS})

# SET(CMAKE_BUILD_TYPE Release)
