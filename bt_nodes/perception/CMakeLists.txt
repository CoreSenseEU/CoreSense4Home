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

# # Python Stuff
# find_package(ament_cmake_python REQUIRED)
# find_package(rclpy REQUIRED)

# # ament_python_install_package(
# #     perception
# #     PACKAGE_DIR script)


set(CMAKE_CXX_STANDARD 17)

set(dependencies
rclcpp
rclpy
behaviortree_cpp_v3
yolov8_msgs
moveit_msgs
shape_msgs
)

include_directories(include)

add_library(extract_object_from_scene_bt_node SHARED src/perception/ExtractObjectsFromScene.cpp)
list(APPEND plugin_libs extract_object_from_scene_bt_node)

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

# ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  script/detection_test.py
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
