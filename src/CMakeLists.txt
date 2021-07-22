cmake_minimum_required(VERSION 3.5.1)
project(obstacle_clustering)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(
  CATKIN_PACKAGE_DEPENDENCIES 
    eigen_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf_conversions
    message_generation
)

find_package(catkin REQUIRED 
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)

###########
# MSGs #
###########

add_message_files(
  FILES
  Obstacle.msg
  Obstacles.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
    ${Eigen_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}_library
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
    message_runtime
)

include_directories(
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

##############
# Executable #
##############

add_executable(clustering src/obstacle_clustering_node.cpp
)

target_link_libraries(clustering ${catkin_LIBRARIES}
)

###########
## Clang ##
###########

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
