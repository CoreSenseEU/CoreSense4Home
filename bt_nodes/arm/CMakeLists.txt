cmake_minimum_required(VERSION 3.8)
project(arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(manipulation_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  behaviortree_cpp_v3
  rclcpp
  rclcpp_cascade_lifecycle
  rclcpp_action
  manipulation_interfaces
  tf2_ros
  geometry_msgs
)

include_directories(include)

add_library(pick_bt_node SHARED src/manipulation/pick_object.cpp)
list(APPEND plugin_libs pick_bt_node)

add_library(move_to_predefined_bt_node SHARED src/manipulation/move_to_predefined.cpp)
list(APPEND plugin_libs move_to_predefined_bt_node)

add_library(point_at_bt_node SHARED src/manipulation/point_at.cpp)
list(APPEND plugin_libs point_at_bt_node)

add_library(place_bt_node SHARED src/manipulation/place_object.cpp)
list(APPEND plugin_libs place_bt_node)

add_library(move_joint_bt_node SHARED src/manipulation/move_joint.cpp)
list(APPEND plugin_libs move_joint_bt_node)

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
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
