cmake_minimum_required(VERSION 3.0.2)
project(pcd_pgm_ros1)

add_compile_options(-std=c++17)
add_definitions(-DPACKAGE_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
)

find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(yaml-cpp REQUIRED)   # <--- 注意这里

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp rospy std_msgs nav_msgs geometry_msgs sensor_msgs
    tf2 tf2_ros tf2_geometry_msgs pcl_ros pcl_conversions cv_bridge
  DEPENDS PCL EIGEN3 OpenCV 
)
include_directories(/usr/include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_definitions(${PCL_DEFINITIONS})

set(SOURCES
  src/pcd2pgm.cpp
  src/Map2D.cpp
  src/YamlReader.cpp
  src/AStarPlanner.cpp
)

add_executable(pcd2pgm_node ${SOURCES})
add_dependencies(pcd2pgm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_compile_definitions(pcd2pgm_node PRIVATE PACKAGE_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(pcd2pgm_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
)

message(STATUS "YAML-CPP include: ${yaml_cpp_INCLUDE_DIRS}")
message(STATUS "YAML-CPP libraries: ${yaml_cpp_LIBRARIES}")
#############
## Install ##
#############

## Mark executables for installation
install(TARGETS pcd2pgm_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark directories for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
