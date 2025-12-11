cmake_minimum_required(VERSION 3.8)
project(mocap_filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# 1. Filter Logic Library
add_library(${PROJECT_NAME}_lib
  src/KalmanFilter.cpp
  src/RhoFilter.cpp
  # Levant is header-only in this setup unless you moved implementation to .cpp
)
target_link_libraries(${PROJECT_NAME}_lib Eigen3::Eigen)

# 2. Single Node Executable
add_executable(filters_node src/filters_node.cpp)
target_link_libraries(filters_node ${PROJECT_NAME}_lib Eigen3::Eigen)
ament_target_dependencies(filters_node 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  tf2 
  tf2_eigen 
  tf2_geometry_msgs
)

# 3. Install
install(TARGETS 
  ${PROJECT_NAME}_lib 
  filters_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()