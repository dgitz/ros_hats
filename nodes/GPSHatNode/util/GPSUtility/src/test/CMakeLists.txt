catkin_add_gtest(test_UTMConversion test_UTMConversion.cpp)
target_link_libraries(test_UTMConversion ${catkin_LIBRARIES} ${Boost_LIBRARIES} utmConversion)