###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  #realtime_tools
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
 add_library(soem   src/soem/ethercatbase.c
                    src/soem/ethercatconfig.c
                    src/soem/ethercatmain.c
                    src/soem/nicdrv.c
                    src/soem/EtherCAT.c
                    src/soem/ethercatdc.c
                    src/soem/ethercatprint.c
                    src/soem/osal.c
                    src/soem/ethercatcoe.c
                    src/soem/ethercatfoe.c
                    src/soem/ethercatsoe.c
 )

## Declare a cpp executable
add_executable(nubot_HWController_node src/nubot_HWController_node.cpp src/cmac.cpp)

add_executable(nubot_teleop_joy src/nubot_teleop_joy.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(nubot_HWController_node ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
 target_link_libraries(nubot_HWController_node
   ${catkin_LIBRARIES} soem ncurses
)

 target_link_libraries(nubot_teleop_joy
   ${catkin_LIBRARIES}
)

