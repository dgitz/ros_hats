include_directories(
 driver/ServoHatDriver/include/
  ${catkin_INCLUDE_DIRS}

)
add_subdirectory(driver)
add_subdirectory(node)