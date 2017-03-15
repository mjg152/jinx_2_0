cmake_minimum_required(VERSION 2.8.3)
project(convert_twist)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs

)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES phidgets
#  CATKIN_DEPENDS roscpp sstream std_msgs stdio stdlib
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(convert_twist src/convert_twist.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(convert_twist ${catkin_LIBRARIES})

