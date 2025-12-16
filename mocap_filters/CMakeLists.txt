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

# 1. Build the library (renamed to match project name for easy linking)
add_library(${PROJECT_NAME}
  src/LevantFilter.cpp
  src/RhoFilter.cpp
  src/rushi_RhoFilter.cpp
  src/NumericalDifferentiation.cpp
  src/DirtyDerivative.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# 2. Build the Node
add_executable(filters_node src/filters_node.cpp)

target_include_directories(filters_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${Eigen3_INCLUDE_DIRS}
)

# Link against the library defined above
target_link_libraries(filters_node ${PROJECT_NAME})

ament_target_dependencies(filters_node 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  tf2
  tf2_ros
  tf2_eigen 
  Eigen3
)

# 3. Export Rules (CRITICAL FOR EXTERNAL LINKING)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3) # Export dependencies needed by public headers
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# 4. Install Rules
install(TARGETS filters_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the library and export it
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY param DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)

ament_package()