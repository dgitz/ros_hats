include_directories(
 driver/GPSHatDriver/include/
  ${catkin_INCLUDE_DIRS}

)
add_subdirectory(util)
add_subdirectory(driver)
add_subdirectory(node)