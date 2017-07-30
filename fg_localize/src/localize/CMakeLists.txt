find_package(Eigen3 REQUIRED) 
include_directories(${EIGEN3_INCLUDE_DIR})
add_executable(localize localize.cpp)
target_link_libraries(localize ${catkin_LIBRARIES})
