add_library(roshat_hat Hat.cpp)
target_link_libraries(roshat_hat ${catkin_LIBRARIES} roshat_channel roshat_port)
install(TARGETS roshat_hat
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
if (${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL armv7l)
        add_library(roshat_relayhat RelayHat.cpp Driver/GPIODriver.cpp)
        target_link_libraries(roshat_relayhat ${catkin_LIBRARIES} logger roshat_hat roshat_digitaloutputchannel roshat_digitaloutputport)


        add_library(roshat_gpshat GPSHat.cpp Driver/GPSHatDriver.cpp)
        target_link_libraries(roshat_gpshat ${catkin_LIBRARIES} gps logger roshat_hat roshat_gpsinputchannel roshat_gpsinputport)
        
        add_library(roshat_servohat ServoHat.cpp Driver/Adafruit16ChServoHat.cpp)
        target_link_libraries(roshat_servohat ${catkin_LIBRARIES} wiringPi logger roshat_hat roshat_servooutputchannel roshat_servooutputport)

        add_library(roshat_terminalhat TerminalHat.cpp Driver/GPIODriver.cpp)
        target_link_libraries(roshat_terminalhat ${catkin_LIBRARIES} wiringPi logger roshat_hat roshat_digitalinputchannel roshat_digitaloutputchannel roshat_digitaloutputport roshat_digitalinputport)


        install(TARGETS roshat_relayhat roshat_gpshat roshat_servohat roshat_terminalhat
                ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
                LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
                RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
endif ()
add_subdirectory(test)
