cmake_minimum_required(VERSION 3.8)
project(core_task_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds models maps rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
