include_directories(include)

add_executable(test_kin
  test/test.cpp
  )
target_link_libraries(test_kin m Eigen3::Eigen)

