add_executable(exec_gps_driver exec_GPSHatDriver.cpp) 
target_link_libraries(exec_gps_driver ${catkin_LIBRARIES} gpsHatDriver )
install(TARGETS exec_gps_driver
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
