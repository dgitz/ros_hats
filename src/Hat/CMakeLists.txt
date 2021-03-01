add_library(roshat_hat Hat.cpp)
target_link_libraries(roshat_hat ${catkin_LIBRARIES} roshat_channel roshat_port)

add_library(roshat_relayhat RelayHat.cpp)
target_link_libraries(roshat_relayhat ${catkin_LIBRARIES} logger roshat_hat roshat_digitaloutputchannel roshat_digitaloutputport)

add_subdirectory(test)

install(TARGETS roshat_hat roshat_relayhat
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
