add_executable(exec_servohat_driver exec_ServoHatDriver.cpp) 
if (${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL armv7l)
    target_link_libraries(exec_servohat_driver ${catkin_LIBRARIES} servoHatDriver )
else()
    target_link_libraries(exec_servohat_driver ${catkin_LIBRARIES} mockServoHatDriver )
endif()


install(TARGETS exec_servohat_driver
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
