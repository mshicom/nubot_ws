#project(world_model)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include)

add_executable(world_model_node
               src/main.cpp
               src/world_model.cpp
               src/ball.cpp
               src/obstacles.cpp
               src/robot.cpp
               src/multicast.cpp)
target_link_libraries(world_model_node  multicast rtdb  ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

#add_executable(world_model_node src/main.cpp)
#target_link_libraries(world_model_node world_model ${catkin_LIBRARIES})
#target_link_libraries(world_model_node rtdb)

