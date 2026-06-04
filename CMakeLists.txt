project(lidar_merge LANGUAGES C CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(livox_msg REQUIRED)
find_package(Eigen3    REQUIRED)
find_package(OpenMP REQUIRED)

fins_add_node(${PROJECT_NAME} 
  node.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  livox_msg::livox_msg
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)

fins_link_ros_dependencies(${PROJECT_NAME}
  visualization_msgs
  sensor_msgs
  geometry_msgs
  tf2_eigen
)