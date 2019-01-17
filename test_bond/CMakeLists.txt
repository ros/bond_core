cmake_minimum_required(VERSION 3.5)
project(test_bond)

# Support C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(bondcpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

pkg_search_module(UUID REQUIRED uuid)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TestBond.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

set(INCLUDE_DIRS
  ${rclcpp_INCLUDE_DIRS}
  ${bondcpp_INCLUDE_DIRS}
)
set(LIBS
  ${rclcpp_LIBRARIES}
  ${bondcpp_LIBRARIES}
  ${UUID_LIBRARIES}
)
include_directories(${INCLUDE_DIRS})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_callbacks_cpp test/test_callbacks_cpp.cpp)
  if(TARGET test_callbacks_cpp)
    target_link_libraries(test_callbacks_cpp ${LIBS} ${GTEST_LIBRARIES})
    rosidl_target_interfaces(test_callbacks_cpp
      ${PROJECT_NAME} "rosidl_typesupport_cpp")
  endif()
  add_executable(test_callbacks_cpp_exc test/test_callbacks_cpp_exc.cpp)
  target_link_libraries(test_callbacks_cpp_exc ${LIBS})
  rosidl_target_interfaces(test_callbacks_cpp_exc
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  add_executable(exercise_bond_cpp_exc test/exercise_bond_cpp_exc.cpp)
  target_link_libraries(exercise_bond_cpp_exc ${LIBS})
  rosidl_target_interfaces(exercise_bond_cpp_exc
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  install(
          TARGETS
          test_callbacks_cpp_exc
          exercise_bond_cpp_exc
          DESTINATION lib/${PROJECT_NAME})

endif()
ament_package()
