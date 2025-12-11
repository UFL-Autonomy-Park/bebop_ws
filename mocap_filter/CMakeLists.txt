cmake_minimum_required(VERSION 3.8)
project(mocap_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# Create KF library
add_library(kf_lib src/KalmanFilter.cpp)
target_include_directories(kf_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kf_lib Eigen3::Eigen)
ament_target_dependencies(kf_lib rclcpp Eigen3)

# Create ROS executable
add_executable(mocap_filter_node src/mocap_filter_node.cpp)
target_include_directories(mocap_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Link libraries
target_link_libraries(mocap_filter_node kf_lib Eigen3::Eigen)
ament_target_dependencies(mocap_filter_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_eigen
  Eigen3
)

# Install rules
install(TARGETS kf_lib mocap_filter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
