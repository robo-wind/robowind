cmake_minimum_required(VERSION 3.0.2)
project(ac_motor_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  ACMotorControl.srv
)

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
add_executable(ac_motor_controller src/ac_motor_controller.cpp)
target_link_libraries(ac_motor_controller ${catkin_LIBRARIES})
add_dependencies(ac_motor_controller ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

## Mark executables for installation
install(TARGETS ac_motor_controller
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
