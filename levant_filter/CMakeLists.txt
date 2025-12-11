cmake_minimum_required(VERSION 3.8)
project(levant_filter)

# Default to C++14 or C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# 2. Add Include Directories
include_directories(include)

# 3. Create Core Library (The Algorithm)
add_library(levant_core src/LevantFilter.cpp)
target_link_libraries(levant_core Eigen3::Eigen)

# 4. Create Node Executable
add_executable(levant_node src/levant_node.cpp)

# --- CRITICAL FIX HERE ---
# Use ament_target_dependencies for ROS packages. 
# This automatically sets include directories and libraries.
ament_target_dependencies(levant_node 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  tf2 
  tf2_geometry_msgs 
  tf2_eigen
)

# Link non-ROS dependencies (Eigen and your internal lib) manually
target_link_libraries(levant_node 
  levant_core
  Eigen3::Eigen
)

# 5. Install Rules
install(TARGETS levant_core levant_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME})

ament_package()