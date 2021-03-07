add_library(roshat_port Port.cpp)
target_link_libraries(roshat_port ${catkin_LIBRARIES} roshat_channel)

add_library(roshat_servooutputport ServoOutputPort.cpp)
target_link_libraries(roshat_servooutputport ${catkin_LIBRARIES} roshat_port roshat_servooutputchannel)

add_library(roshat_digitaloutputport DigitalOutputPort.cpp)
target_link_libraries(roshat_digitaloutputport ${catkin_LIBRARIES} roshat_port roshat_digitaloutputchannel)

add_library(roshat_digitalinputport DigitalInputPort.cpp)
target_link_libraries(roshat_digitalinputport ${catkin_LIBRARIES} roshat_port roshat_digitalinputchannel)

add_library(roshat_gpsinputport GPSInputPort.cpp)
target_link_libraries(roshat_gpsinputport ${catkin_LIBRARIES} roshat_port roshat_gpsinputchannel)

install(TARGETS roshat_port roshat_digitaloutputport roshat_servooutputport roshat_digitalinputport roshat_gpsinputport
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
