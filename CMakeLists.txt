cmake_minimum_required(VERSION 2.8.3)
project(dolphin_slam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  message_generation
  cv_bridge
  sensor_msgs
  geometry_msgs
  image_transport
  visualization_msgs
  underwater_sensor_msgs
  time_monitor
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LocalViewCell.msg
  LocalViewNetwork.msg
  PoseCellActivity.msg
  ExperienceEvent.msg
  Error.msg
  ExecutionTime.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   RobotPose.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
 #LIBRARIES dolphin_slam
  CATKIN_DEPENDS roscpp tf message_runtime
  DEPENDS system_lib OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(dolphin_slam
#   src/${PROJECT_NAME}/dolphin_slam.cpp
# )

#target_link_libraries(dolphin_slam ${catkin_LIBRARIES} )

## Declare a cpp executable
add_executable(local_view_node
    src/${PROJECT_NAME}/bag_of_words.cpp
    src/${PROJECT_NAME}/local_view_module.cpp
    src/local_view_node.cpp
)

add_executable(experience_map_node
    src/${PROJECT_NAME}/experience_map.cpp
    src/experience_map_node.cpp
)

add_executable(pose_cell_node
    src/${PROJECT_NAME}/pose_cell_network.cpp
    src/pose_cell_node.cpp
)

add_executable(robot_state_node
    src/${PROJECT_NAME}/robot_state.cpp
    src/robot_state_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(local_view_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(experience_map_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(pose_cell_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(robot_state_node ${catkin_LIBRARIES} ${OpenCV_LIBS})


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(local_view_node dolphin_slam_generate_messages_cpp)
add_dependencies(experience_map_node dolphin_slam_generate_messages_cpp)
add_dependencies(pose_cell_node dolphin_slam_generate_messages_cpp)
add_dependencies(robot_state_node dolphin_slam_generate_messages_cpp)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS dolphin_slam dolphin_slam_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dolphin_slam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
