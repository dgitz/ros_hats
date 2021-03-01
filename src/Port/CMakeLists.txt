add_library(roshat_port Port.cpp)
target_link_libraries(roshat_port ${catkin_LIBRARIES} roshat_channel)

add_library(roshat_pwmoutputport PWMOutputPort.cpp)
target_link_libraries(roshat_pwmoutputport ${catkin_LIBRARIES} roshat_port roshat_pwmoutputchannel)

add_library(roshat_digitaloutputport DigitalOutputPort.cpp)
target_link_libraries(roshat_digitaloutputport ${catkin_LIBRARIES} roshat_port roshat_digitaloutputchannel)

install(TARGETS roshat_port roshat_pwmoutputport roshat_digitaloutputport
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
