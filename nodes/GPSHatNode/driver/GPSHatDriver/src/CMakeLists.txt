add_library(gpsHatDriver GPSHatDriver.cpp)
target_link_libraries(gpsHatDriver ${catkin_LIBRARIES} gps logger prettyUtility convertUtility)
add_subdirectory(test)