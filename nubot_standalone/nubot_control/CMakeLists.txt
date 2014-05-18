include_directories(include)
add_executable(nubot_control_node src/nubot_control.cpp
                                  src/behaviour.cpp
                                  src/plan.cpp
                                  src/strategy.cpp

)

target_link_libraries(nubot_control_node
   ${catkin_LIBRARIES}
 )


