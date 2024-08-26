include_directories(../)
include_directories(../kinematics/include)
include_directories(../2R_kinematics/include)
include_directories(../power_monitor/include)
include_directories(include)

add_executable(test
  test/test.cpp
  src/trajectory_planner.cpp
  )
target_link_libraries(test m pthread)

add_executable(test_withIK
  test/test_withIK.cpp
  src/trajectory_planner.cpp
  )
target_link_libraries(test_withIK m pthread)
 
add_executable(test_bpc_traj
  test/test_bpc_traj.cpp
  src/trajectory_planner.cpp
  )
target_link_libraries(test_bpc_traj m pthread)

add_executable(test_tptraj_BPC_2R
  test/test_tptraj_BPC_2R.cpp
  src/trajectory_planner.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  )
target_link_libraries(test_tptraj_BPC_2R m pthread bcm_host)

add_executable(test_tptraj_BPC_2R_all_freq
  test/test_tptraj_BPC_2R_all_freq.cpp
  src/trajectory_planner.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  ../power_monitor/src/smbus.cpp
  )
target_link_libraries(test_tptraj_BPC_2R_all_freq m pthread bcm_host)

add_executable(test_TTBPC
  test/test_TTBPC.cpp
  ../trajectory_planner/src/trajectory_planner.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  )
target_link_libraries(test_TTBPC m pthread bcm_host)

add_executable(test_dynamic_tracking_plot
  test/test_dynamic_tracking_plot.cpp
  ../trajectory_planner/src/trajectory_planner.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  ../power_monitor/src/smbus.cpp
  )
target_link_libraries(test_dynamic_tracking_plot m pthread bcm_host)

add_executable(test_dynamic_tracking_socket_comm1
  test/test_dynamic_tracking_socket_comm.cpp
  ../trajectory_planner/src/trajectory_planner.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  ../power_monitor/src/smbus.cpp
  )
target_link_libraries(test_dynamic_tracking_socket_comm1 m pthread bcm_host)

add_executable(test_dynamic_tracking_socket_comm
  src/test_dynamic_tracking_socket_comm.cpp
  ../trajectory_planner/src/trajectory_planner.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  ../power_monitor/src/smbus.cpp
  )
target_link_libraries(test_dynamic_tracking_socket_comm m pthread bcm_host)

###

