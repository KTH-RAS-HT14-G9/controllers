cmake_minimum_required(VERSION 2.8.3)
project(motor_controller)

find_package(catkin REQUIRED COMPONENTS roscpp)
# std_msgs)
find_package(common REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

catkin_package()

###########
## Build ##
###########

include_directories(${common_INCLUDE_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS}
					"../etc")

add_executable(motor_controller motor_controller.cpp)
target_link_libraries(motor_controller ${catkin_LIBRARIES})
target_link_libraries(motor_controller ${common_LIBRARIES})

#############
## Install ##
#############


#############
## Testing ##
#############

