cmake_minimum_required(VERSION 3.0.2)
project(robobus)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
   FILES
   SerialRead.msg
)

add_service_files(
  FILES
  SerialWrite.srv
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
)

## Declare a C++ executable
add_executable(robobus src/robobus.cpp)
target_link_libraries(robobus ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables for installation
install(TARGETS robobus
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
