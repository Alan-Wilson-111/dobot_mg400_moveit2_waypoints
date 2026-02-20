cmake_minimum_required(VERSION 3.8)
project(mg400_description)

# Find ament_cmake
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  launch
  meshes
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# This is required at the end
ament_package()
