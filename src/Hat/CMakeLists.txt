add_library(roshat_hat Hat.cpp)
target_link_libraries(roshat_hat ${catkin_LIBRARIES} roshat_channel roshat_port)

install(TARGETS roshat_hat
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
