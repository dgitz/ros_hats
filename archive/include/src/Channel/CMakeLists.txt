add_library(roshat_channel Channel.cpp)
target_link_libraries(roshat_channel ${catkin_LIBRARIES})

add_library(roshat_servooutputchannel ServoOutputChannel.cpp)
target_link_libraries(roshat_servooutputchannel ${catkin_LIBRARIES} roshat_channel)

add_library(roshat_digitaloutputchannel DigitalOutputChannel.cpp)
target_link_libraries(roshat_digitaloutputchannel ${catkin_LIBRARIES} roshat_channel)

add_library(roshat_digitalinputchannel DigitalInputChannel.cpp)
target_link_libraries(roshat_digitalinputchannel ${catkin_LIBRARIES} roshat_channel)

add_library(roshat_gpsinputchannel GPSInputChannel.cpp)
target_link_libraries(roshat_gpsinputchannel ${catkin_LIBRARIES} roshat_channel)

install(TARGETS roshat_channel roshat_servooutputchannel roshat_digitaloutputchannel roshat_digitalinputchannel roshat_gpsinputchannel
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
