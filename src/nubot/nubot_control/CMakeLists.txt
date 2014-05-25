cmake_minimum_required(VERSION 2.8.3)
project(nubot_control)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")



find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  std_srvs
  nubot_common
)

#generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_generation 
	         roscpp 
	         rospy  
		 std_msgs 
		 std_srvs 
		 DEPENDS
)

set(nubot_control_includes
               ${catkin_INCLUDE_DIRS}
               ${PROJECT_SOURCE_DIR}/include
)

include_directories(${nubot_control_includes})



add_executable(nubot_control_node src/nubot_control.cpp
                                  src/behaviour.cpp
                                  src/plan.cpp
                                  src/strategy.cpp

)

target_link_libraries(nubot_control_node
   ${catkin_LIBRARIES}
 )


