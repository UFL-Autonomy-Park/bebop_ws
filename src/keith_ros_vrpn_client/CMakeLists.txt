#		cmake_minimum_required(VERSION 2.6)
#		include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#
#		# Set the build type.  Options are:
#		#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#		#  Debug          : w/ debug symbols, w/o optimization
#		#  Release        : w/o debug symbols, w/ optimization
#		#  RelWithDebInfo : w/ debug symbols, w/ optimization
#		#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#		#set(ROS_BUILD_TYPE RelWithDebInfo)
#
#		rosbuild_init()
#
#		#set the default path for built executables to the "bin" directory
#		set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#		#set the default path for built libraries to the "lib" directory
#		set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#
#		#uncomment if you have defined messages
#		#genmsg()
#		#rosbuild_genmsg()
#		#uncomment if you have defined services
#		#gensrv()
#
#		#common commands for building c++ executables and libraries
#		#rospack_add_library(${PROJECT_NAME} src/example.cpp)
#		#target_link_libraries(${PROJECT_NAME} another_library)
#		#rospack_add_boost_directories()
#		#rospack_link_boost(${PROJECT_NAME} thread)
#		#rospack_add_executable(example examples/example.cpp)
#		#target_link_libraries(example ${PROJECT_NAME})
#
#		rosbuild_add_executable(ros_vrpn_client src/ros_vrpn_client.cpp)
#		target_link_libraries(ros_vrpn_client vrpn)
#		#target_link_libraries(vrpn ${PROJECT_SOURCE_DIR}/vrpn/pc_linux)
#
# cmake_minimum_required(VERSION 2.8.3)
# project(ros_vrpn_client)
#
# find_package(ament REQUIRED COMPONENTS roscpp tf geometry_msgs)
#
# ament_package(
#	INCLUDE_DIRS vrpn
# )
#
#
# include_directories(vrpn)
# include_directories(
#	${catkin_INCLUDE_DIRS}
#)
#
# add_executable(ros_vrpn_client src/ros_vrpn_client.cpp
#	vrpn/vrpn_Connection.C vrpn/vrpn_Tracker.C vrpn/vrpn_Shared.C vrpn/vrpn_FileConnection.C
#	vrpn/vrpn_BaseClass.C vrpn/vrpn_Serial.C)
# target_link_libraries(ros_vrpn_client ${catkin_LIBRARIES})
cmake_minimum_required(VERSION 3.5)
project(ros_vrpn_client)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Threads REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include vrpn vrpn/include vrpn/quat Threads rclcpp)

add_executable(ros_vrpn_client src/ros_vrpn_client.cpp vrpn/vrpn_Thread.C vrpn/vrpn_EndpointContainer.C vrpn/quat/vector.c vrpn/quat/matrix.c vrpn/quat/quat.c
	vrpn/vrpn_Connection.C vrpn/vrpn_Tracker.C vrpn/vrpn_Shared.C vrpn/vrpn_FileConnection.C
	vrpn/vrpn_BaseClass.C vrpn/vrpn_Serial.C vrpn/quat/quat.c)
target_link_libraries(ros_vrpn_client pthread)
ament_target_dependencies(ros_vrpn_client rclcpp tf2 tf2_ros geometry_msgs)

ament_package()

install(TARGETS
        ros_vrpn_client
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
        )



