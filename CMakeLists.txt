cmake_minimum_required(VERSION 2.8.3)
project(restorebot_moveit_desc)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#this is the junk that I included (below). Pray this works

#the following is from moveit_tutorial's main CMakeLists.txt

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_experimental
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
)

find_package(Boost REQUIRED system filesystem date_time thread)

#note: I removed the include of doc/interactivity
#might need to remove LIBRARIES interactivity_utils

catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_experimental
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

#the following is added from motion_planning_api
add_executable(motion_planning_restorebot src/motion_planning_restorebot.cpp)
target_link_libraries(motion_planning_restorebot ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS motion_planning_restorebot DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(chomp_planning_restorebot src/chomp_planning_restorebot.cpp)
target_link_libraries(chomp_planning_restorebot ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS chomp_planning_restorebot DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
