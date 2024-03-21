cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(yolov8_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(perception_system REQUIRED)


set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclpy
  behaviortree_cpp_v3
  yolov8_msgs
  moveit_msgs
  shape_msgs
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  tf2
  rclcpp_action
  behaviortree_cpp_v3
  perception_system
)

include_directories(include)

add_library(extract_object_from_scene_bt_node SHARED src/perception/ExtractObjectsFromScene.cpp)
list(APPEND plugin_libs extract_object_from_scene_bt_node)

add_library(is_detected_bt_node SHARED src/perception/IsDetected.cpp)
list(APPEND plugin_libs is_detected_bt_node)

add_library(follow_person_bt_node SHARED src/perception/FollowPerson.cpp)
list(APPEND plugin_libs follow_person_bt_node)

add_library(extract_collision_scene_bt_node SHARED src/perception/extract_collision_scene.cpp)
list(APPEND plugin_libs extract_collision_scene_bt_node)

add_library(is_entity_moving_bt_node SHARED src/perception/is_entity_moving.cpp)
list(APPEND plugin_libs is_entity_moving_bt_node)

add_library(is_moving_bt_node SHARED src/perception/is_moving.cpp)
list(APPEND plugin_libs is_moving_bt_node)

add_library(filter_entity_bt_node SHARED src/perception/filter_entity.cpp)
list(APPEND plugin_libs filter_entity_bt_node)

add_library(is_pointing_bt_node SHARED src/perception/is_pointing.cpp)
list(APPEND plugin_libs is_pointing_bt_node)

foreach(bt_plugin ${plugin_libs})
ament_target_dependencies(${bt_plugin} ${dependencies})
target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS
${plugin_libs}
EXPORT ${PROJECT_NAME}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
DESTINATION include/
)

install(PROGRAMS
  script/detection_test.py
  script/attention_test.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
