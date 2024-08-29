include_directories(../)
include_directories(../kinematics/include)
include_directories(../trajectory_planner/include)
include_directories(../power_monitor/include)

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

add_executable(test_line_trap_traj
  src/test_line_trap_traj.cpp
  ../trajectory_planner/src/trajectory_planner.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  )
target_link_libraries(test_line_trap_traj m pthread bcm_host)

add_executable(test_op_mode_switch
  src/test_op_mode_switch.cpp
  ../trajectory_planner/src/trajectory_planner.cpp
  ../pi3hat/pi3hat.cc
  ../pi3hat/pi3hat_interface.cpp
  )
target_link_libraries(test_op_mode_switch m pthread bcm_host)

#add_executable(test_max_loading
#  src/test_max_loading.cpp
#  ../trajectory_planner/src/trajectory_planner.cpp
#  ../pi3hat/pi3hat.cc
#  ../pi3hat/pi3hat_interface.cpp
#  ../power_monitor/src/smbus.cpp
#  )
#target_link_libraries(test_max_loading m pthread bcm_host)

#add_executable(test_velocity_ellipse
#  src/test_velocity_ellipse.cpp
#  ../trajectory_planner/src/trajectory_planner.cpp
#  ../pi3hat/pi3hat.cc
#  ../pi3hat/pi3hat_interface.cpp
#  )
#target_link_libraries(test_velocity_ellipse m pthread bcm_host)

###

