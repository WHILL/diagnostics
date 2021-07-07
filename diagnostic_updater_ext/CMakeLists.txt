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

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
  INTERFACE
    include
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  INTERFACE
    ${catkin_LIBRARIES}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
