cmake_minimum_required(VERSION 3.8)
project(g1_base_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(nav_obstacle_cloud_filter_core
  src/nav_obstacle_cloud_filter.cpp
)
target_compile_features(nav_obstacle_cloud_filter_core PUBLIC cxx_std_17)
target_include_directories(nav_obstacle_cloud_filter_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(nav_obstacle_cloud_filter_core
  Eigen3::Eigen
)
ament_target_dependencies(nav_obstacle_cloud_filter_core
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(nav_obstacle_cloud_filter
  src/nav_obstacle_cloud_filter_node.cpp
)
target_compile_features(nav_obstacle_cloud_filter PUBLIC cxx_std_17)
target_link_libraries(nav_obstacle_cloud_filter
  nav_obstacle_cloud_filter_core
)
ament_target_dependencies(nav_obstacle_cloud_filter
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  nav_obstacle_cloud_filter_core
  nav_obstacle_cloud_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_nav_obstacle_cloud_filter
    test/test_nav_obstacle_cloud_filter.cpp
  )
  target_compile_features(test_nav_obstacle_cloud_filter PUBLIC cxx_std_17)
  target_include_directories(test_nav_obstacle_cloud_filter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_nav_obstacle_cloud_filter
    nav_obstacle_cloud_filter_core
    Eigen3::Eigen
  )
  ament_target_dependencies(test_nav_obstacle_cloud_filter
    nav_msgs
    rclcpp
    sensor_msgs
    std_msgs
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(nav_obstacle_cloud_filter_core)
ament_export_dependencies(
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

ament_package()
