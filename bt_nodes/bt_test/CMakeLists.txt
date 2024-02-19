cmake_minimum_required(VERSION 3.8)
project(bt_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(whisper_msgs REQUIRED)
find_package(configuration REQUIRED)
find_package(motion REQUIRED)
find_package(perception REQUIRED)
find_package(hri REQUIRED)
find_package(backward_ros REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  audio_common_msgs
  whisper_msgs
  configuration
  motion
  perception
  hri
  backward_ros
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

# Speak test
add_executable(speak_test src/speak_test.cpp)
ament_target_dependencies(speak_test ${dependencies})
target_link_libraries(speak_test hri::speak_bt_node hri::dialogConfirmation_bt_node)
# ExtractObjectFromScene test
add_executable(extract_object_from_scene_test src/extract_object_from_scene_test.cpp)
ament_target_dependencies(extract_object_from_scene_test ${dependencies})
target_link_libraries(extract_object_from_scene_test perception::extract_object_from_scene_bt_node)

# GenerateTextFromObjets test
add_executable(generate_text_from_objects_test src/generate_text_from_objects_test.cpp)
ament_target_dependencies(generate_text_from_objects_test ${dependencies})
target_link_libraries(generate_text_from_objects_test hri::generate_text_from_objects_bt_node)

install(TARGETS
  set_wp_test
  move_to_test
  setwp_and_moveto_test
  speak_test
  extract_object_from_scene_test
  generate_text_from_objects_test
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
