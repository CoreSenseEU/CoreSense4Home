cmake_minimum_required(VERSION 3.8)
project(cs4home_core)

set(CMAKE_BUILD_TYPE debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/cs4home_core/Master.cpp
  src/cs4home_core/CognitiveModule.cpp
  src/cs4home_core/Afferent.cpp
  src/cs4home_core/Efferent.cpp
  src/cs4home_core/Core.cpp
  src/cs4home_core/Meta.cpp
  src/cs4home_core/Coupling.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
