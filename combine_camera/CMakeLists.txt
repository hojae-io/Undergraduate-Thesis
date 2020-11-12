cmake_minimum_required(VERSION 2.8.3)
project(combine_camera)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

catkin_package(
  LIBRARIES combine_camera
  CATKIN_DEPENDS std_msgs roscpp sensor_msgs cv_bridge
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


 add_executable(combine_camera src/cam_concat.cpp)

 add_dependencies(combine_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(combine_camera
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )

