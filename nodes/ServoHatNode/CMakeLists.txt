include_directories(
      driver/include/
      ${catkin_INCLUDE_DIRS}
    )
if (${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL armv7l)
    include_directories(
      driver/ServoHatDriver/include/
      ${catkin_INCLUDE_DIRS}
    )
else()
    include_directories(
      driver/MockServoHatDriver/include/
      ${catkin_INCLUDE_DIRS}
    )
endif()

add_subdirectory(driver)
#add_subdirectory(node)