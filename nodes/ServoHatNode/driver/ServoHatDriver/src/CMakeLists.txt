add_library(servoHatDriver ServoHatDriver.cpp)
target_link_libraries(servoHatDriver ${catkin_LIBRARIES} logger prettyUtility convertUtility wiringPi)
add_subdirectory(test)