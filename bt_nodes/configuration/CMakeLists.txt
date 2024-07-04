cmake_minimum_required(VERSION 3.8)
project(configuration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_cascade_lifecycle
  rclcpp_action
  behaviortree_cpp_v3
  geometry_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
)

include_directories(include)

add_library(set_wp_bt_node SHARED src/configuration/SetWp.cpp)
list(APPEND plugin_libs set_wp_bt_node)

add_library(init_carry_bt_node SHARED src/configuration/init_carry.cpp)
list(APPEND plugin_libs init_carry_bt_node)

add_library(init_receptionist_bt_node SHARED src/configuration/init_receptionist.cpp)
list(APPEND plugin_libs init_receptionist_bt_node)

add_library(add_guest_to_count_bt_node SHARED src/configuration/add_guest_to_count.cpp)
list(APPEND plugin_libs add_guest_to_count_bt_node)

add_library(set_start_position_bt_node SHARED src/configuration/SetStartPosition.cpp)
list(APPEND plugin_libs set_start_position_bt_node)

add_library(deferred_bt_node SHARED src/configuration/Deferred.cpp)
list(APPEND plugin_libs deferred_bt_node)

add_library(setup_gpsr_bt_node SHARED src/configuration/Setup_gpsr.cpp)
list(APPEND plugin_libs setup_gpsr_bt_node)
add_library(init_groceries_bt_node SHARED src/configuration/init_groceries.cpp)
list(APPEND plugin_libs init_groceries_bt_node)

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
