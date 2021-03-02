add_library(hatNodeProcess HatNodeProcess.cpp)
target_link_libraries(hatNodeProcess ${catkin_LIBRARIES} baseNodeProcess)
add_subdirectory(test)

if (${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL armv7l)

    add_executable(hat_node HatNode.cpp) 
    target_link_libraries(hat_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} hatNodeProcess baseNode roshat_relayhat roshat_pwmhat)
    add_dependencies(hat_node ${eros_EXPORTED_TARGETS})

endif ()