catkin_add_gtest(test_GPSHatNodeProcess test_GPSHatNodeProcess.cpp)
target_link_libraries(test_GPSHatNodeProcess ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger gpsHatNodeProcess)
add_dependencies(test_GPSHatNodeProcess eros_generate_messages_cpp)

add_rostest_gtest(test_GPSHatNode
    test_GPSHatNode.test
    test_GPSHatNode.cpp
)
add_dependencies(test_GPSHatNode gps_hat_node)
target_link_libraries(test_GPSHatNode ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger)
