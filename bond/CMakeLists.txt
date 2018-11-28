cmake_minimum_required(VERSION 3.5)

project(bond)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Constants.msg"
  "msg/Status.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

#add_message_files(DIRECTORY msg
#  FILES
#  Constants.msg
#  Status.msg
#)

#generate_messages(DEPENDENCIES std_msgs)

#catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
ament_package()

