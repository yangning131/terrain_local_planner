cmake_minimum_required(VERSION 2.8.3)
project(fitting_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    std_msgs
    std_srvs
   # slam_navi_msgs
    tf
    costmap_2d
    base_local_planner
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(glog 0.3.4 REQUIRED)
find_package(Boost REQUIRED)

SET(INC_DIR "/usr/local/include/")
SET(LINK_DIR "/usr/local/lib")
 
#INCLUDE_DIRECTORIES(${glog_DIR})
INCLUDE_DIRECTORIES(${INC_DIR})
LINK_DIRECTORIES(${LINK_DIR})
 
###################################
## catkin specific configuration ##
###################################
catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES ur_msgs
   CATKIN_DEPENDS
   roscpp
   message_runtime
   std_msgs
   std_srvs
   tf
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    include
    ${Eigen_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
src/global_planner.cpp
src/planner/bezier_path_smoother.cpp
src/planner/bezier_path.cpp
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
# LINK_LIBRARIES(log4cplus)
target_link_libraries(
    ${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
)
#############
## Install ##
#############

#############
## Testing ##
#############
