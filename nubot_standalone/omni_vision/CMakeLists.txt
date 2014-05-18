include_directories(include)
add_executable(omni_vision_node src/omnivision.cpp
      src/localization.cpp  
      src/omniimage.cpp         
      src/scanpoints.cpp
      src/fieldinformation.cpp  
      src/obstacles.cpp     
      src/optimise.cpp          
      src/transfer.cpp
      src/glocalization.cpp     
      src/odometry.cpp      
      src/robotinformation.cpp  
      src/whites.cpp
      src/colorsegment.cpp
      src/ballfinder.cpp)

target_link_libraries(omni_vision_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )

