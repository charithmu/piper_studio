cmake_minimum_required(VERSION 3.8)
project(piper_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/wait_for_clock.py scripts/gz_server.sh scripts/export_urdf.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_models test/test_models.py TIMEOUT 120)
  ament_add_pytest_test(test_mujoco_model test/test_mujoco_model.py TIMEOUT 300)
  ament_add_pytest_test(test_servo_gains test/test_servo_gains.py TIMEOUT 300)
endif()

ament_package()
