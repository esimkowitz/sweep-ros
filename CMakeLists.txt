cmake_minimum_required(VERSION 3.5)
project(sweep_ros)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -Wpedantic")

find_package(ament_cmake REQUIRED)
find_package(Sweep REQUIRED)
find_package(Threads REQUIRED)
find_package(PCL REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)

add_executable(sweep_node src/node.cpp)
target_compile_options(sweep_node PRIVATE -Wall)
ament_target_dependencies(sweep_node rclcpp sensor_msgs pcl_conversions pcl_msgs)
target_link_libraries(sweep_node ${PCL_LIBRARIES} ${LIBSWEEP_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})

target_include_directories(sweep_node SYSTEM PRIVATE ${LIBSWEEP_INCLUDE_DIR} ${ament_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS sweep_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Tell downstream packages where to find our headers
ament_export_include_directories(include)
# Tell downstream packages our libraries to link against
# ament_export_libraries(my_library)
# Help downstream packages to find transitive dependencies
ament_export_dependencies(
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_msgs
  Sweep
  Threads
  PCL
)

ament_package()