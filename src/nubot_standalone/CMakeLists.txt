cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


#set(CMAKE_BUILD_TYPE "DEBUG")
#set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "possible configurations" FORCE)
# In case the user does not setup CMAKE_BUILD_TYPE, assume it's RelWithDebInfo
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "build type default to RelWithDebInfo, set to Release to improve performance" FORCE)
endif("${CMAKE_BUILD_TYPE}" STREQUAL "")

project(nubot_standalone)
 
find_package(Boost  REQUIRED )
find_package(catkin REQUIRED )
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)
find_package(catkin REQUIRED COMPONENTS roscpp 
  sensor_msgs 
  nav_msgs 
  message_filters 
  image_transport 
  cv_bridge 
  image_geometry 
  tf 
  dynamic_reconfigure
  realtime_tools
)

#find_package(Eigen  REQUIRED)

add_message_files(DIRECTORY msg FILES Info.msg
                                      DebugInfo.msg
                                      Angle.msg
                                      Point2d.msg
                                      PPoint.msg
                                      Point3d.msg
                                      RobotInfo.msg
                                      BallInfo.msg
                                      ObstaclesInfo.msg
                                      OminiVisionInfo.msg
                                      VelCmd.msg
                                      CoachInfo.msg
                                      WorldModelInfo.msg
)

add_service_files(DIRECTORY srv FILES BallHandle.srv
                                      Shoot.srv
)

generate_messages(DEPENDENCIES std_msgs)


generate_dynamic_reconfigure_options(
  nubot_HWController/cfg/controller.cfg
)


#some common includes
set(nubot_standalone_includes 
               ${catkin_INCLUDE_DIRS} 
               ${PROJECT_SOURCE_DIR}/core/include
               ${PROJECT_SOURCE_DIR}/rtdb/include
            #  ${PROJECT_SOURCE_DIR}/eigen/eigen3.1.3
            #  ${PROJECT_SOURCE_DIR}/common/include
)

include_directories(${nubot_standalone_includes})


catkin_package(CATKIN_DEPENDS message_runtime std_msgs opencv2/ roscpp)

#add_subdirectory(omni_vision)
#add_subdirectory(world_model)
#add_subdirectory(rtdb)
#add_subdirectory(motion_drive)
#add_subdirectory(nubot_control)

FILE(GLOB_RECURSE new_list *.h)
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path}/../../../include)
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
INCLUDE_DIRECTORIES(${dir_list})


file( GLOB list "*" )
foreach( entry ${list} )
  if( IS_DIRECTORY ${entry} )
    if( EXISTS ${entry}/CMakeLists.txt )
      add_subdirectory( ${entry} )
      link_directories( ${entry}/lib )
      message( STATUS "Configuring  ${entry} xiao" )
    endif()
  endif()
endforeach()
