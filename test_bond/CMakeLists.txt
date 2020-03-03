cmake_minimum_required(VERSION 3.0.2)

project(test_bond)

find_package(catkin REQUIRED COMPONENTS cmake_modules bondcpp message_generation rostest)

if(CATKIN_ENABLE_TESTING)
  include_directories(${GTEST_INCLUDE_DIRS})
  include_directories(include ${bondcpp_INCLUDE_DIRS})

  # create the service
  add_service_files(DIRECTORY srv FILES TestBond.srv)
  generate_messages()
endif()

catkin_package()

if(CATKIN_ENABLE_TESTING)
  # add more tests
  find_package(UUID REQUIRED)
  include_directories(${UUID_INCLUDE_DIRS})
  add_executable(exercise_bond EXCLUDE_FROM_ALL test/exercise_bond.cpp)
  target_link_libraries(exercise_bond ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${UUID_LIBRARIES})
  if(test_bond_EXPORTED_TARGETS)
    add_dependencies(exercise_bond ${test_bond_EXPORTED_TARGETS})
  endif()
  add_dependencies(tests exercise_bond)

  add_rostest(test/test_cpp.launch)
  add_rostest(test/test_python.launch)

  add_executable(test_callbacks_cpp EXCLUDE_FROM_ALL test/test_callbacks_cpp.cpp)
  target_link_libraries(test_callbacks_cpp ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${UUID_LIBRARIES})
  if(test_bond_EXPORTED_TARGETS)
    add_dependencies(test_callbacks_cpp ${test_bond_EXPORTED_TARGETS})
  endif()
  add_dependencies(tests test_callbacks_cpp)

  add_rostest(test/test_callbacks_cpp.launch)
  add_rostest(test/test_callbacks_py.launch)
endif()
