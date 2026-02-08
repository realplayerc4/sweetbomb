cmake_minimum_required(VERSION 3.8)
project(octomap_web_viewer)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install web files
install(DIRECTORY web/
  DESTINATION share/${PROJECT_NAME}/web
)

# Install scripts
install(PROGRAMS scripts/start_web_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME start_web_server
)

install(PROGRAMS scripts/pointcloud_voxelizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pointcloud_voxelizer
)

ament_package()
