cmake_minimum_required(VERSION 3.8)
project(piper_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/command_guard.py scripts/wait_for_clock.py scripts/gz_server.sh DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_mujoco_model test/test_mujoco_model.py TIMEOUT 300)
  ament_add_pytest_test(test_servo_gains test/test_servo_gains.py TIMEOUT 300)
endif()

ament_package()
