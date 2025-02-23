add_library(servoHatDriver ServoHatDriver.cpp)
target_link_libraries(servoHatDriver ${catkin_LIBRARIES} logger prettyUtility convertUtility wiringPi)
# No tests here, as this is hardware dependent