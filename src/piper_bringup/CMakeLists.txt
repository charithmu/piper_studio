cmake_minimum_required(VERSION 3.8)
project(piper_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/command_guard.py DESTINATION lib/${PROJECT_NAME})

ament_package()
