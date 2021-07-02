find_package(rostest REQUIRED)

add_rostest_gtest(${PROJECT_NAME}_test
  diagnostic_updater_test.xml
  diagnostic_updater_test.cpp
)

target_link_libraries(${PROJECT_NAME}_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_rostest(diagnostic_updater_fast_test.xml)
