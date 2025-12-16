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
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# 1. Build the library (Filters logic)
add_library(filter_libs 
  src/LevantFilter.cpp
  src/RhoFilter.cpp
  src/rushi_RhoFilter.cpp
)

# Allow the library to find headers in 'include'
target_include_directories(filter_libs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)

# Link Eigen to the library
target_link_libraries(filter_libs Eigen3::Eigen)

# 2. Build the Node
add_executable(filters_node src/filters_node.cpp)

target_include_directories(filters_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(filters_node filter_libs)

ament_target_dependencies(filters_node 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  tf2
  tf2_ros
  tf2_eigen 
  Eigen3
)

# 3. Install Rules
install(TARGETS filters_node filter_libs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# Standard header install location
install(DIRECTORY include/ DESTINATION include/)

ament_package()