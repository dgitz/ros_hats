include_directories(
 driver/GPSHatDriver/include/
 util/
  ${catkin_INCLUDE_DIRS}

)
add_subdirectory(util)
add_subdirectory(driver)
add_subdirectory(node)