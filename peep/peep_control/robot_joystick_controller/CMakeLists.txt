cmake_minimum_required(VERSION 3.0.2)
project(robot_joystick_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  dc_motor_controller
)

################################################
## Declare ROS messages, services and actions ##
################################################

#add_service_files(
 # FILES
#)

generate_messages(
   DEPENDENCIES
   std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################



###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ executable
add_executable(robot_joystick_controller src/robot_joystick_controller.cpp)
target_link_libraries(robot_joystick_controller ${catkin_LIBRARIES})
add_dependencies(robot_joystick_controller ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

## Mark executables for installation
install(TARGETS robot_joystick_controller
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robobus.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
#   Action1.action
#   Action2.action
# )



