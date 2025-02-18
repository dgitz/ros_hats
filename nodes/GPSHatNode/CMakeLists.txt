include_directories(
 driver/GPSHatDriver/include/
  ${catkin_INCLUDE_DIRS}

)
add_subdirectory(driver)
add_subdirectory(node)