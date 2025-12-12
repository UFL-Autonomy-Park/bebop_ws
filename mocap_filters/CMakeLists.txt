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
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(filter_libs 
  src/LevantFilter.cpp
  src/RhoFilter.cpp
  src/KalmanFilter.cpp
)
target_link_libraries(filter_libs Eigen3::Eigen)

add_executable(filters_node src/filters_node.cpp)
target_link_libraries(filters_node filter_libs)
ament_target_dependencies(filters_node 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  tf2 
  tf2_eigen 
  Eigen3
)

install(TARGETS filters_node filter_libs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION share/${PROJECT_NAME})

ament_package()