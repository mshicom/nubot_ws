find_package( Threads )
add_executable(comm multicast.cpp comm.cpp)
target_link_libraries(comm ${CMAKE_THREAD_LIBS_INIT} ${PROJECT_NAME})

add_library(multicast multicast.cpp)
