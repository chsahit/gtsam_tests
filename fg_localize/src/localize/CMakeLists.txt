find_package(Eigen3 REQUIRED GTSAM TBB) 
include_directories(${EIGEN3_INCLUDE_DIR})
add_executable(localize localize.cpp)
target_link_libraries(localize ${catkin_LIBRARIES} /usr/local/lib/libgtsam.so /usr/lib/x86_64-linux-gnu/libtbb.so)
