catkin_add_gtest(test_ServoHatNodeProcess test_ServoHatNodeProcess.cpp)
target_link_libraries(test_ServoHatNodeProcess ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger servoHatNodeProcess)
add_dependencies(test_ServoHatNodeProcess eros_generate_messages_cpp)

add_rostest_gtest(test_ServoHatNode
    test_ServoHatNode.test
    test_ServoHatNode.cpp
)
add_dependencies(test_ServoHatNode servo_hat_node)
target_link_libraries(test_ServoHatNode ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger)
