add_library(hatDefinition HatDefinition.cpp)
target_link_libraries(hatDefinition ${catkin_LIBRARIES})

add_library(raspberryPiDefinition RaspberryPiDefinition.cpp)
target_link_libraries(raspberryPiDefinition ${catkin_LIBRARIES})
add_subdirectory(test)