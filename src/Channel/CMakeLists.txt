add_library(roshat_channel Channel.cpp)
target_link_libraries(roshat_channel ${catkin_LIBRARIES})

add_library(roshat_pwminputchannel PWMInputChannel.cpp)
target_link_libraries(roshat_pwminputchannel ${catkin_LIBRARIES} roshat_channel)

add_library(roshat_pwmoutputchannel PWMOutputChannel.cpp)
target_link_libraries(roshat_pwmoutputchannel ${catkin_LIBRARIES} roshat_channel)

add_library(roshat_digitaloutputchannel DigitalOutputChannel.cpp)
target_link_libraries(roshat_digitaloutputchannel ${catkin_LIBRARIES} roshat_channel)

install(TARGETS roshat_channel roshat_pwminputchannel roshat_pwmoutputchannel roshat_digitaloutputchannel
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
