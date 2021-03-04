catkin_add_gtest(test_roshat_definitions test_definitions.cpp)
target_link_libraries(test_roshat_definitions ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(test_roshat_definitions ros_hats_generate_messages_cpp)

catkin_add_gtest(test_roshat_base test_base.cpp)
target_link_libraries(test_roshat_base ${catkin_LIBRARIES} ${Boost_LIBRARIES} 
    roshat_pwmoutputchannel 
    roshat_digitaloutputchannel 
    roshat_digitaloutputchannel 
    roshat_pwmoutputport
    roshat_digitaloutputport)
