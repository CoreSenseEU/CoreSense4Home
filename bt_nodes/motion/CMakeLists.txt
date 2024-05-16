cmake_minimum_required(VERSION 3.8)
project(motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(attention_system REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(navigation_system_interfaces REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_cascade_lifecycle
  rclcpp_action
  behaviortree_cpp_v3
  geometry_msgs
  nav2_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  attention_system
  nav2_costmap_2d
  pluginlib
  navigation_system_interfaces
  slam_toolbox
  ament_index_cpp
  trajectory_msgs
  lifecycle_msgs
)

include_directories(include)

add_library(clear_people_layer SHARED
            src/motion/navigation/nav2_plugins/clear_people_layer.cpp)
ament_target_dependencies(clear_people_layer ${dependencies})

add_library(move_to_bt_node SHARED src/motion/navigation/MoveTo.cpp)
list(APPEND plugin_libs move_to_bt_node)

add_library(look_at_bt_node SHARED src/motion/head/LookAt.cpp)
list(APPEND plugin_libs look_at_bt_node)

add_library(follow_entity_bt_node SHARED src/motion/navigation/follow_entity.cpp)
list(APPEND plugin_libs follow_entity_bt_node)

add_library(configure_navigate_back_bt_node SHARED src/motion/navigation/configure_navigate_back.cpp)
list(APPEND plugin_libs configure_navigate_back_bt_node)

add_library(clear_map_at_goal_bt_node SHARED src/motion/navigation/clear_map_at_goal.cpp)
list(APPEND plugin_libs clear_map_at_goal_bt_node)

add_library(pan_bt_node SHARED src/motion/head/Pan.cpp)
list(APPEND plugin_libs pan_bt_node)

add_library(goal_publisher_bt_node SHARED src/motion/navigation/goal_publisher.cpp)
list(APPEND plugin_libs goal_publisher_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS
  ${plugin_libs}
  clear_people_layer
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)
install(FILES clear_people_layer.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(clear_people_layer)
ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
pluginlib_export_plugin_description_file(nav2_costmap_2d clear_people_layer.xml)

ament_package()
