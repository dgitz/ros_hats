add_library(utmConversion UTMConversion.cpp)
target_link_libraries(utmConversion ${catkin_LIBRARIES})
add_subdirectory(test)