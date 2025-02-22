add_library(servoHatNodeProcess ServoHatNodeProcess.cpp)
#target_link_libraries(servoHatNodeProcess ${catkin_LIBRARIES} baseNodeProcess gpsHatDriver)

add_executable(servo_hat_node ServoHatNode.cpp) 
target_link_libraries(servo_hat_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} servoHatNodeProcess baseNode   )
add_dependencies(servo_hat_node ${eros_EXPORTED_TARGETS})
#add_subdirectory(test)
