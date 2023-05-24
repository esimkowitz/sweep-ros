cmake_minimum_required(VERSION 3.5)
project(sweep_ros)

find_package(ament_cmake REQUIRED)

find_package(Sweep REQUIRED)
find_package(Threads REQUIRED)
find_package(PCL REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wconversion -Wno-sign-conversion -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

# start sweep_ros library
add_library(sweep_ros SHARED
  src/sweep_ros.cpp
)
target_compile_options(sweep_ros
  PRIVATE -Wall
)

ament_target_dependencies(sweep_ros
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_msgs
)

target_link_libraries(sweep_ros ${PCL_LIBRARIES} ${LIBSWEEP_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})
target_include_directories(sweep_ros SYSTEM PRIVATE ${LIBSWEEP_INCLUDE_DIR} ${ament_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
# end sweep_ros library

# start sweep_node executable
add_executable(sweep_node
  src/sweep_node.cpp
)
target_compile_options(sweep_node
  PRIVATE -Wall
)
target_link_libraries(sweep_node
  sweep_ros
)
ament_target_dependencies(sweep_node
  rclcpp
)
# end sweep_node executable

install(TARGETS
  sweep_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  sweep_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install (DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  include/
  DESTINATION include
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