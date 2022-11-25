cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

# add_library(
#   str_functions src/str_functions.c
# )

# ament_package(CATKIN_DEPENDS std_msgs
#                DEPENDS Boost
#                INCLUDE_DIRS include
#                LIBRARIES your_library)




# target_link_libraries(${executable_name} str_functions)

add_executable(talker src/publisher_member_function.cpp)

ament_target_dependencies(talker
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "visualization_msgs"
  # "rcl_interfaces"
)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()