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

add_executable(tf_remap src/tf_remapper_node.cpp)
add_dependencies(tf_remap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_remap ${PROJECT_NAME} ${catkin_LIBRARIES})

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
  add_rostest(test/test_tf_remapper_node_empty_mappings.launch)
  add_rostest(test/test_tf_remapper_node_works_if_no_mappings.launch)
  add_rostest(test/test_tf_remapper_node_static.launch)
  add_rostest(test/test_tf_remapper_node_static_with_param.launch)
  add_rostest(test/test_tf_remapper_node_bidi_forward.launch)
  add_rostest(test/test_tf_remapper_node_bidi_back.launch)
  add_rostest(test/test_tf_remapper_node_bidi_both.launch)
  add_rostest(test/test_tf_remapper_node_bidi_both_static.launch)

  # In coverage build, you can choose either lcov or gcovr to generate the reports
  if ("${CMAKE_BUILD_TYPE}" STREQUAL "Coverage")
    include(cmake/CodeCoverage.cmake)
    APPEND_COVERAGE_COMPILER_FLAGS()
    if(${COVERAGE_GCOVR})
      set(COVERAGE_GCOVR_EXCLUDES "'/opt/ros/*'" "'/usr/*'" "test/*" ${COVERAGE_GCOVR_ADDITIONAL_EXCLUDES})
      SETUP_TARGET_FOR_COVERAGE_GCOVR_HTML(NAME ${PROJECT_NAME}_coverage EXECUTABLE test_tf_remapper)
      add_dependencies(${PROJECT_NAME}_coverage test_tf_remapper_node check_tf_exists tf_remap run_tests)
    else()
      set(COVERAGE_LCOV_EXCLUDES "'/opt/ros/*'" "'/usr/*'" "test/*" ${COVERAGE_LCOV_ADDITIONAL_EXCLUDES})
      SETUP_TARGET_FOR_COVERAGE_LCOV(NAME ${PROJECT_NAME}_coverage EXECUTABLE test_tf_remapper DEPENDENCIES run_tests)
      add_dependencies(${PROJECT_NAME}_coverage test_tf_remapper_node check_tf_exists tf_remap)
    endif()

    add_custom_target(upload_coverage_to_codecov
        COMMAND curl -s -o codecov.bash https://codecov.io/bash
        COMMAND bash codecov.bash || echo "Codecov did not collect coverage reports"
        COMMAND ${CMAKE_COMMAND} -E remove codecov.bash
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
        COMMENT "Uploading codecov data."
        )
  endif()
endif()

# Docs

add_custom_target(docs
    COMMAND rosdoc_lite .
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Generating documentaiton")

add_custom_target(upload_docs
    COMMAND
      mkdir -p /tmp/${PROJECT_NAME}-docs &&
      cd /tmp/${PROJECT_NAME}-docs &&
      rm -rf /tmp/${PROJECT_NAME}-docs/${PROJECT_NAME} &&
      git clone -b gh-pages --depth 1 git@github.com:tradr-project/${PROJECT_NAME}.git &&
      cd ${PROJECT_NAME} &&
      touch .nojekyll &&
      rsync -aAP ${PROJECT_SOURCE_DIR}/doc/html/ . &&
      git add --all &&
      git commit -m "Docs build" &&
      git push origin gh-pages &&
      cd /tmp
    COMMAND rm -rf /tmp/${PROJECT_NAME}-docs/${PROJECT_NAME}
    WORKING_DIRECTORY /tmp
    COMMENT "Uploading documentation"
    )
add_dependencies(upload_docs docs)

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME} tf_remap
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