add_library(hatNodeProcess HatNodeProcess.cpp)
target_link_libraries(hatNodeProcess ${catkin_LIBRARIES} baseNodeProcess)

add_executable(hat_node HatNode.cpp) 
target_link_libraries(hat_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} hatNodeProcess baseNode roshat_relayhat)
add_dependencies(hat_node ${eros_EXPORTED_TARGETS})
add_subdirectory(test)
