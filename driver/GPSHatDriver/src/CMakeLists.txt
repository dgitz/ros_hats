add_library(gpsHatDriver GPSHatDriver.cpp)
target_link_libraries(gpsHatDriver ${catkin_LIBRARIES} gps logger)
#add_dependencies(gpsHatDriver eros_generate_messages_cpp)