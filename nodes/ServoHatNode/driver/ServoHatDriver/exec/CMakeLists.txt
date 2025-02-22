add_executable(exec_servohat_driver exec_ServoHatDriver.cpp) 
target_link_libraries(exec_servohat_driver ${catkin_LIBRARIES} servoHatDriver )
install(TARGETS exec_servohat_driver
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
