cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
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
find_package(OpenCV REQUIRED)
find_package(perception_system_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hri REQUIRED)
find_package(hri_msgs REQUIRED)


set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_cascade_lifecycle
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
  OpenCV
  perception_system_interfaces
  sensor_msgs
  std_srvs
  hri
  hri_msgs
)

include_directories(include)

add_library(extract_object_from_scene_bt_node SHARED src/perception/ExtractObjectsFromScene.cpp)
list(APPEND plugin_libs extract_object_from_scene_bt_node)

add_library(convert_color_bt_node SHARED src/perception/convert_color.cpp)
list(APPEND plugin_libs convert_color_bt_node)

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

add_library(extract_entity_color_bt_node SHARED src/perception/extract_entity_color.cpp)
list(APPEND plugin_libs extract_entity_color_bt_node)

add_library(is_sittable_bt_node SHARED src/perception/is_sittable.cpp)
list(APPEND plugin_libs is_sittable_bt_node)

add_library(is_door_open_bt_node SHARED src/perception/IsDoorOpen.cpp)
list(APPEND plugin_libs is_door_open_bt_node)

add_library(extract_cabinet_place_pose_bt_node SHARED src/perception/extract_cabinet_place_pose.cpp)
list(APPEND plugin_libs extract_cabinet_place_pose_bt_node)

add_library(is_gesturing_bt_node SHARED src/perception/is_gesturing.cpp)
list(APPEND plugin_libs is_gesturing_bt_node)

add_library(count_people_bt_node SHARED src/perception/count_people.cpp)
list(APPEND plugin_libs count_people_bt_node)

add_library(filter_object_bt_node SHARED src/perception/filter_object.cpp)
list(APPEND plugin_libs filter_object_bt_node)

add_library(filter_prev_detections_bt_node SHARED src/perception/filter_prev_detections.cpp)
list(APPEND plugin_libs filter_prev_detections_bt_node)

add_library(extract_person_description_bt_node SHARED src/perception/extract_person_description.cpp)
list(APPEND plugin_libs extract_person_description_bt_node)
add_library(clear_octomap_bt_node SHARED src/perception/clear_octomap.cpp)
list(APPEND plugin_libs clear_octomap_bt_node)

add_library(switch_yolo_model_bt_node SHARED src/perception/switch_yolo_model.cpp)
list(APPEND plugin_libs switch_yolo_model_bt_node)

add_library(extract_pc_from_class_bt_node SHARED src/perception/extract_pc_from_class.cpp)
list(APPEND plugin_libs extract_pc_from_class_bt_node)

add_library(is_face_detected_bt_node SHARED src/perception/is_face_detected.cpp)
list(APPEND plugin_libs is_face_detected_bt_node)

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
