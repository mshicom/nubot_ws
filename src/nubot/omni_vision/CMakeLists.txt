cmake_minimum_required(VERSION 2.8.3)
project(omni_vision)
#  std=gnu++0x" enables c++ 11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")

find_package(catkin REQUIRED COMPONENTS
  nubot_common
  cv_bridge
  dynamic_reconfigure
  image_transport
  message_filters
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(cfg/OmniVision.cfg)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport message_filters message_generation opencv2/ roscpp rospy sensor_msgs std_msgs std_srvs
)

set(omni_vision_includes
               ${catkin_INCLUDE_DIRS}
               ${PROJECT_SOURCE_DIR}/include
               ${Boost_INCLUDE_DIRS}
               ${Eigen_INCLUDE_DIRS}
)

include_directories(${omni_vision_includes})
add_executable(omni_vision_node
      src/omnivision.cpp
      src/localization.cpp  
      src/omniimage.cpp         
      src/scanpoints.cpp
      src/fieldinformation.cpp
      src/optimise.cpp          
      src/transfer.cpp
      src/glocalization.cpp     
      src/odometry.cpp      
      src/robotinformation.cpp  
      src/whites.cpp
      src/colorsegment.cpp
      src/ballfinder.cpp
      src/iAuction.cpp
      src/MTTracker.cpp
      src/obstacles.cpp
)

target_link_libraries(omni_vision_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS omni_vision omni_vision_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_omni_vision.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
