find_package(Eigen3 REQUIRED GTSAM TBB) 
include_directories(${EIGEN3_INCLUDE_DIR})
add_executable(model model.cpp)
target_link_libraries(model ${catkin_LIBRARIES} /usr/local/lib/libgtsam.so /usr/lib/x86_64-linux-gnu/libtbb.so)
