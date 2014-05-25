cmake_minimum_required(VERSION 2.8.3)
project(world_model)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")

find_package(catkin REQUIRED COMPONENTS
  nubot_common
  dynamic_reconfigure
  message_filters
  message_generation
  roscpp
  rospy
  std_msgs
  std_srvs
)
 
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dynamic_reconfigure message_filters message_generation  roscpp rospy  std_msgs std_srvs
)

set(world_model_includes
               ${catkin_INCLUDE_DIRS}
               ${PROJECT_SOURCE_DIR}/include
)

include_directories(${world_model_includes})

add_executable(world_model_node
               src/main.cpp
               src/world_model.cpp
               src/ball.cpp
               src/obstacles.cpp
               src/robot.cpp
               src/multicast.cpp)

target_link_libraries(world_model_node
    ${catkin_LIBRARIES}
)
