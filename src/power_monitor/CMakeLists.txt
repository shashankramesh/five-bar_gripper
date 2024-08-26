cmake_minimum_required(VERSION 3.0.2)
project(power_monitor)

## Compile as C++11 with -O2 optimisation
add_compile_options(-O2 -std=c++11)
set(CMAKE_INSTALL_PREFIX ".")

include_directories(include)

add_executable(power_monitor_test
  src/smbus.cpp
  src/power_monitor_test.cpp
  )
target_link_libraries(power_monitor_test)
