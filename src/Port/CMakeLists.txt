add_library(roshat_port Port.cpp)
target_link_libraries(roshat_port ${catkin_LIBRARIES} roshat_channel)

add_library(roshat_pwmoutputport PWMOutputPort.cpp)
target_link_libraries(roshat_pwmoutputport ${catkin_LIBRARIES} roshat_port roshat_pwmoutputchannel)

install(TARGETS roshat_port roshat_pwmoutputport
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
