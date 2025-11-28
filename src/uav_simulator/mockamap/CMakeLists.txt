cmake_minimum_required(VERSION 3.8)
project(mockamap)

# Compile options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# set(PCL_DIR "/usr/local/share/pcl-1.14")
# find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED date_time)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# include directories
include_directories(
  include/mockamap
  ${rclcpp_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)

ament_export_include_directories(include)

# Source file
file(GLOB ${PROJECT_NAME}_SRCS src/*.cpp)

# Create an executable file
add_executable(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_SRCS}
)


# Link library
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  pcl_ros
  pcl_conversions
  nav_msgs
  sensor_msgs
  visualization_msgs
  Boost
)

# Install executable file
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

#Install launch file
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
# Install config (the config file containing rviz)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

#Install header files
# install(DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )

ament_package()
