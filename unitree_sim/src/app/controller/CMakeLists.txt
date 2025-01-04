cmake_minimum_required(VERSION 3.16)
project(unitree_controller)

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SIM_DEPENDENCIES
    pthread
    mujoco::mujoco
    glfw
    yaml-cpp
    # unitree_sdk2
)


add_executable(unitree_controller main.cc lowlevel_controller.cc)
target_include_directories(unitree_controller PRIVATE ${UNITREE_INCLUDE_DIRS})
target_link_libraries(unitree_controller ${SIM_DEPENDENCIES} unitreesim_common)
ament_target_dependencies(unitree_controller ${DEPENDENCY_LIST})

add_executable(sdk2_controller sdk2_main.cc)
target_include_directories(sdk2_controller PRIVATE ${UNITREE_INCLUDE_DIRS})
target_link_libraries(sdk2_controller ${SIM_DEPENDENCIES})
target_link_libraries(sdk2_controller unitree_sdk2)

# SET(CMAKE_BUILD_TYPE Release)