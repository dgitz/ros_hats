add_library(servoHatNodeProcess ServoHatNodeProcess.cpp)
if (${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL armv7l)
    target_link_libraries(servoHatNodeProcess ${catkin_LIBRARIES} baseNodeProcess servoHatDriver)
else()
    target_link_libraries(servoHatNodeProcess ${catkin_LIBRARIES} baseNodeProcess mockServoHatDriver)
endif()


add_executable(servo_hat_node ServoHatNode.cpp) 
target_link_libraries(servo_hat_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} servoHatNodeProcess baseNode   )
add_dependencies(servo_hat_node ${eros_EXPORTED_TARGETS})
add_subdirectory(test)
