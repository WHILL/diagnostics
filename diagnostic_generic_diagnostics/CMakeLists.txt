cmake_minimum_required(VERSION 2.8.3)
project(diagnostic_generic_diagnostics)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  diagnostic_updater_ext
  nodelet
  topic_tools
  std_msgs
  diagnostic_msgs
  )

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS bind)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    nodelet
    topic_tools
    std_msgs
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## node
add_executable(topic_monitor
  src/topic_monitor_node.cpp)
add_dependencies(topic_monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(topic_monitor ${catkin_LIBRARIES})

add_executable(bool_monitor
  src/bool_monitor_node.cpp)
add_dependencies(bool_monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bool_monitor ${catkin_LIBRARIES})

## nodelet
add_library(topic_monitor_nodelet
  src/topic_monitor_nodelet.cpp)
add_dependencies(topic_monitor_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(topic_monitor_nodelet ${catkin_LIBRARIES})

add_library(bool_monitor_nodelet
  src/bool_monitor_nodelet.cpp)
add_dependencies(bool_monitor_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bool_monitor_nodelet ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS
        topic_monitor
        topic_monitor_nodelet
        bool_monitor
        bool_monitor_nodelet
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(topic_monitor_nodetest test/launch/topic_monitor_nodetest.test test/topic_monitor_nodetest.cpp)
  target_link_libraries(topic_monitor_nodetest ${catkin_LIBRARIES})

  add_rostest_gtest(topic_monitor_nodelettest test/launch/topic_monitor_nodelettest.test test/topic_monitor_nodetest.cpp)
  target_link_libraries(topic_monitor_nodelettest ${catkin_LIBRARIES})

  add_rostest_gtest(bool_monitor_nodetest test/launch/bool_monitor_nodetest.test test/bool_monitor_nodetest.cpp)
  target_link_libraries(bool_monitor_nodetest ${catkin_LIBRARIES})

  add_rostest_gtest(bool_monitor_nodelettest test/launch/bool_monitor_nodelettest.test test/bool_monitor_nodetest.cpp)
  target_link_libraries(bool_monitor_nodelettest ${catkin_LIBRARIES})

  add_rostest(test/launch/bool_monitor_hztest_off_timer.test)
  add_rostest(test/launch/bool_monitor_hztest_on_timer.test)

  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
