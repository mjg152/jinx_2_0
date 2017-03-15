cmake_minimum_required(VERSION 2.8.3)
project(phidgets)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  phidgets_api
  libphidgets
  tf
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  encoder_params.msg
  manager_params.msg

)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES phidgets
#  CATKIN_DEPENDS roscpp sstream std_msgs stdio stdlib
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(odometry src/odometry.cpp)
add_executable(encoder src/encoder.cpp)



## Specify libraries to link a library or executable target against
target_link_libraries(odometry ${catkin_LIBRARIES})
target_link_libraries(encoder ${catkin_LIBRARIES})
