cmake_minimum_required(VERSION 3.5)
project(practica1_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(dependencies
    rclcpp
    std_msgs
    rclcpp_lifecycle
    sensor_msgs
    geometry_msgs
    lifecycle_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/practica1_pkg/WallFollower.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(follow_wall_main
  src/follow_wall_main.cpp
)
ament_target_dependencies(follow_wall_main ${dependencies})
target_link_libraries(follow_wall_main ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  follow_wall_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

ament_export_dependencies(${dependencies})
ament_package()