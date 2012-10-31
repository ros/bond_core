cmake_minimum_required(VERSION 2.8.3)
project(test_bond)

find_package(catkin REQUIRED COMPONENTS bondcpp genmsg rostest)

include_directories(${GTEST_INCLUDE_DIRS})
include_directories(SYSTEM ${bondcpp_INCLUDE_DIRS})

# create the service
add_service_files(DIRECTORY srv FILES TestBond.srv)
generate_messages()

catkin_package()

# add more tests
add_executable(exercise_bond EXCLUDE_FROM_ALL test/exercise_bond.cpp)
target_link_libraries(exercise_bond ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
add_dependencies(exercise_bond ${PROJECT_NAME}_gencpp)
add_dependencies(tests exercise_bond)

add_rostest(test/test_cpp.launch)
add_rostest(test/test_python.launch)

add_executable(test_callbacks_cpp EXCLUDE_FROM_ALL test/test_callbacks_cpp.cpp)
target_link_libraries(test_callbacks_cpp ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
add_dependencies(test_callbacks_cpp ${PROJECT_NAME}_gencpp)
add_dependencies(tests test_callbacks_cpp)

add_rostest(test/test_callbacks_cpp.launch)
add_rostest(test/test_callbacks_py.launch)
