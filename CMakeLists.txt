if(WIN32)
  cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
else()
  cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
endif()

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

project (Gazebo)
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
string (TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

# Find Gazebo
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(gazebo_wheel_slip_plugin SHARED gazebo_wheel_slip_plugin.cc)
target_link_libraries(gazebo_wheel_slip_plugin ${GAZEBO_LIBRARIES})

ament_target_dependencies(gazebo_wheel_slip_plugin
  "gazebo"
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
)

# Build ht_nav plugin
add_library(ht_nav_gazebo_wheel_slip_plugin SHARED ht_nav_gazebo_wheel_slip_plugin.cc)
target_link_libraries(ht_nav_gazebo_wheel_slip_plugin ${GAZEBO_LIBRARIES})

ament_target_dependencies(ht_nav_gazebo_wheel_slip_plugin
  "gazebo"
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
)
