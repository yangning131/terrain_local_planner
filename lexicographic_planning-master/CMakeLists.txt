cmake_minimum_required(VERSION 3.10)
project(lexicographic_planning)




include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_FLAGS} doesn't have C++11 support.
                         Please use a different C++ compiler")
endif ()

set(ADDITIONAL_CXX_FLAG "-Wall -O3  -pthread")
set(CMAKE_CXX_FLAGS "${OpenMP_CXX_FLAGS} ${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG} ")

# set(CMAKE_CXX_FLAGS "${OpenMP_CXX_FLAGS} -std=c++11 -O3 -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  # pcl library
  kdtree1
  pcl_ros
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  pluginlib
  nav_core
  cv_bridge
  image_transport
)

find_package(OpenMP)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED)




catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL

  CATKIN_DEPENDS 
  geometry_msgs 
  std_msgs
  nav_msgs
  visualization_msgs
  kdtree1
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

  
)
include(cmake/glog.cmake)
include_directories(${GLOG_INCLUDE_DIRS})

include(cmake/FindEigen.cmake)
include_directories(${EIGEN_INCLUDE_DIR})

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
)

###########
## Build ##
###########

# Server
add_executable(${PROJECT_NAME}_pathServer src/pathServer.cpp)
target_link_libraries(${PROJECT_NAME}_pathServer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Planner
add_executable(${PROJECT_NAME}_pathPlanning src/pathPlanning.cpp
                                                                                                         src/node3d.cpp
                                                                                                         src/visualize.cpp
                                                                                                         src/collisiondetection.cpp
                                                                                                         src/path.cpp
                                                                                                         src/rs_path.cpp
                                                                                                         )
target_link_libraries(${PROJECT_NAME}_pathPlanning ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${GLOG_LIBRARIES})

# Obstable Server
add_executable(${PROJECT_NAME}_obstacleServer src/obstacleServer.cpp)
target_link_libraries(${PROJECT_NAME}_obstacleServer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Cloud transformer
add_executable(${PROJECT_NAME}_cloudRegister src/cloudRegister.cpp)
target_link_libraries(${PROJECT_NAME}_cloudRegister ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# path map  generater
add_executable(${PROJECT_NAME}_pathmap src/pathmap.cpp)
target_link_libraries(${PROJECT_NAME}_pathmap ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# move_base Planner Plugin
add_library(lex_planner_lib src/library/lex_planner.cpp)
target_link_libraries(lex_planner_lib ${catkin_LIBRARIES})