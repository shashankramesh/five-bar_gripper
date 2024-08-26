include_directories(../)
include_directories(../kinematics/include)
include_directories(../trajectory_planner/include)
include_directories(../power_monitor/include)

add_executable(test_position_signs
  src/test_position_signs.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  )
target_link_libraries(test_position_signs m pthread bcm_host)

add_executable(test_FK
  src/test_FK.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  )
target_link_libraries(test_FK m pthread bcm_host)

add_executable(test_IK
  src/test_IK.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  )
target_link_libraries(test_IK m pthread bcm_host)

add_executable(test_sinetraj_IK
  src/test_sinetraj_IK.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  )
target_link_libraries(test_sinetraj_IK m pthread bcm_host)

add_executable(test_Singularity
  src/test_Singularity.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  )
target_link_libraries(test_Singularity m pthread bcm_host)

add_executable(test_line_trap_traj
  src/test_line_trap_traj.cpp
  ../trajectory_planner/src/trajectory_planner.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  )
target_link_libraries(test_line_trap_traj m pthread bcm_host)

add_executable(test_bpc_trap_traj
  src/test_bpc_trap_traj.cpp
  ../trajectory_planner/src/trajectory_planner.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  )
target_link_libraries(test_bpc_trap_traj m pthread bcm_host)

add_executable(test_ss_error
  src/test_ss_error.cpp
  ../trajectory_planner/src/trajectory_planner.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  )
target_link_libraries(test_ss_error m pthread bcm_host)

add_executable(test_max_tracking_freq
  src/test_max_tracking_freq.cpp
  ../trajectory_planner/src/trajectory_planner.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  )
target_link_libraries(test_max_tracking_freq m pthread bcm_host)

add_executable(test_pow_con
  src/test_pow_con.cpp
  ../trajectory_planner/src/trajectory_planner.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  )
target_link_libraries(test_pow_con m pthread bcm_host)

add_executable(test_max_loading
  src/test_max_loading.cpp
  ../trajectory_planner/src/trajectory_planner.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  ../power_monitor/src/smbus.cpp
  )
target_link_libraries(test_max_loading m pthread bcm_host)

add_executable(test_velocity_ellipse
  src/test_velocity_ellipse.cpp
  ../trajectory_planner/src/trajectory_planner.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  )
target_link_libraries(test_velocity_ellipse m pthread bcm_host)

add_executable(test_settling_time
  src/test_settling_time.cpp
  ../trajectory_planner/src/trajectory_planner.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  )
target_link_libraries(test_settling_time m pthread bcm_host)

###

