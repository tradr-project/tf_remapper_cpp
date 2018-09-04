cmake_minimum_required(VERSION 2.8.3)
project(tf_remapper_cpp)

find_package(catkin REQUIRED COMPONENTS
  xmlrpcpp
  roscpp
  tf2_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS xmlrpcpp roscpp tf2_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/tf_remapper.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/tf_remapper_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

if(${CATKIN_ENABLE_TESTING})
  catkin_add_gtest(test_tf_remapper test/test_tf_remapper.cpp)
  add_dependencies(test_tf_remapper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_tf_remapper ${PROJECT_NAME} ${catkin_LIBRARIES})

  find_package(rostest REQUIRED)
  find_package(tf2_ros REQUIRED)

  add_executable(test_tf_remapper_node test/test_tf_remapper_node.cpp)
  target_link_libraries(test_tf_remapper_node ${catkin_LIBRARIES} ${tf2_ros_LIBRARIES} ${GTEST_LIBRARIES})
  add_executable(check_tf_exists test/check_tf_exists.cpp)
  target_link_libraries(check_tf_exists ${catkin_LIBRARIES} ${tf2_ros_LIBRARIES} ${GTEST_LIBRARIES})
  add_rostest(test/test_tf_remapper_node.launch)
  add_rostest(test/test_tf_remapper_node_static.launch)
  add_rostest(test/test_tf_remapper_node_bidi_forward.launch)
  add_rostest(test/test_tf_remapper_node_bidi_back.launch)
  add_rostest(test/test_tf_remapper_node_bidi_both.launch)

  # In coverage build, you can choose either lcov or gcovr to generate the reports
  if ("${CMAKE_BUILD_TYPE}" STREQUAL "Coverage")
    include(cmake/CodeCoverage.cmake)
    APPEND_COVERAGE_COMPILER_FLAGS()
    if(${COVERAGE_GCOVR})
      set(COVERAGE_GCOVR_EXCLUDES "'/opt/ros/*'" "'/usr/*'" "test/*" "tf_remapper_node.cpp" "test_tf_remapper_node.cpp" "tf_remapper_node.h" ${COVERAGE_GCOVR_ADDITIONAL_EXCLUDES})
      SETUP_TARGET_FOR_COVERAGE_GCOVR_HTML(NAME ${PROJECT_NAME}_coverage EXECUTABLE test_tf_remapper)
    else()
      set(COVERAGE_LCOV_EXCLUDES "'/opt/ros/*'" "'/usr/*'" "test/*" "*/tf_remapper_node.cpp" "*/test_tf_remapper_node.cpp" "*/tf_remapper_node.h" ${COVERAGE_LCOV_ADDITIONAL_EXCLUDES})
      SETUP_TARGET_FOR_COVERAGE_LCOV(NAME ${PROJECT_NAME}_coverage EXECUTABLE test_tf_remapper)
    endif()
  endif()
endif()

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)