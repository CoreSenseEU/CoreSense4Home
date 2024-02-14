cmake_minimum_required(VERSION 3.8)
project(bt_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(configuration REQUIRED)
find_package(motion REQUIRED)
find_package(perception REQUIRED)
find_package(tf2_ros REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  configuration
  motion
  perception
  tf2_ros
)

# Set WP test
add_executable(set_wp_test src/set_wp_test.cpp)
ament_target_dependencies(set_wp_test ${dependencies})
target_link_libraries(set_wp_test configuration::set_wp_bt_node)

# Move To test
add_executable(move_to_test src/move_to_test.cpp)
ament_target_dependencies(move_to_test ${dependencies})
target_link_libraries(move_to_test motion::move_to_bt_node)

# Set WP and MoveTo test
add_executable(setwp_and_moveto_test src/setwp_and_moveto_test.cpp)
ament_target_dependencies(setwp_and_moveto_test ${dependencies})
target_link_libraries(setwp_and_moveto_test configuration::set_wp_bt_node motion::move_to_bt_node)

# Is Detected test
add_executable(is_detected_test src/is_detected_test.cpp)
ament_target_dependencies(is_detected_test ${dependencies})
target_link_libraries(is_detected_test perception::is_detected_bt_node)

# Is Detected Real test
add_executable(is_detected_real_test src/is_detected_real_test.cpp)
ament_target_dependencies(is_detected_real_test ${dependencies})
target_link_libraries(is_detected_real_test perception::is_detected_bt_node)

# Is Detected Move To test
add_executable(isdetected_moveto_test src/isdetected_moveto_test.cpp)
ament_target_dependencies(isdetected_moveto_test ${dependencies})
target_link_libraries(isdetected_moveto_test perception::is_detected_bt_node motion::move_to_bt_node)

install(TARGETS
  set_wp_test
  move_to_test
  setwp_and_moveto_test
  is_detected_test
  is_detected_real_test
  isdetected_moveto_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY bt_xml config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})

ament_package()
