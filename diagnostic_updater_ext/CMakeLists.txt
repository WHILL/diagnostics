cmake_minimum_required(VERSION 2.8.3)
project(diagnostic_updater_ext)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED
  diagnostic_msgs
  diagnostic_updater
  roscpp
  std_msgs
)

catkin_package(
  DEPENDS
    diagnostic_msgs
    diagnostic_updater
    roscpp
    std_msgs
  INCLUDE_DIRS include
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
