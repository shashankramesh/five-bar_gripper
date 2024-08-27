cmake_minimum_required(VERSION 3.0.2)
project(five_bar_gripper)

## Compile as C++11 with -O2 optimisation
add_compile_options(-O2 -std=c++11)
set(CMAKE_INSTALL_PREFIX ".")

find_package(Eigen3 REQUIRED)
include_directories(./)
include_directories(/opt/vc/include)
link_directories(/opt/vc/lib)

add_subdirectory(src/kinematics)
add_subdirectory(src/tests)
#add_subdirectory(src/trajectory_planner)
#add_subdirectory(src/power_monitor)

###

