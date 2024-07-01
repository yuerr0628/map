cmake_minimum_required(VERSION 3.0.2)
project(parking_slot_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  cv_bridge
  image_transport
)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  gcn_parking.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   sensor_msgs
)

find_package(OpenCV 3 REQUIRED COMPONENTS core imgcodecs imgproc calib3d)
find_package(Eigen3 3.3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES parking_slot_detection
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(client_test_node
#  src/client_test.cpp 
# )

add_executable(client_test_node
  src/main.cpp 
  src/pose.cpp
  src/drawmap.cpp
  src/associate.cpp
  src/Hungarian.cpp
)

target_link_libraries(client_test_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

add_dependencies(client_test_node  parking_slot_detection_generate_messages_cpp)

