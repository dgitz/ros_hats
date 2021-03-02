catkin_add_gtest(test_relayhat test_relayhat.cpp)
target_link_libraries(test_relayhat ${catkin_LIBRARIES} ${Boost_LIBRARIES} roshat_relayhat)

catkin_add_gtest(test_pwmhat test_pwmhat.cpp)
target_link_libraries(test_pwmhat ${catkin_LIBRARIES} ${Boost_LIBRARIES} roshat_pwmhat)
