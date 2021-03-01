catkin_add_gtest(test_relayhat test_relayhat.cpp)
target_link_libraries(test_relayhat ${catkin_LIBRARIES} ${Boost_LIBRARIES} roshat_relayhat)
