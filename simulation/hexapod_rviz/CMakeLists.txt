cmake_minimum_required(VERSION 3.10)
project(hexapod_rviz)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf stl rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
