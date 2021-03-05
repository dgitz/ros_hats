catkin_add_gtest(test_relayhat test_relayhat.cpp)
target_link_libraries(test_relayhat ${catkin_LIBRARIES} ${Boost_LIBRARIES} roshat_relayhat)

catkin_add_gtest(test_terminalhat test_terminalhat.cpp)
target_link_libraries(test_terminalhat ${catkin_LIBRARIES} ${Boost_LIBRARIES} roshat_terminalhat)

if (${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL armv7l)
    catkin_add_gtest(test_servohat test_servohat.cpp)
    target_link_libraries(test_servohat ${catkin_LIBRARIES} ${Boost_LIBRARIES} roshat_servohat)
endif ()
