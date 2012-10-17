cmake_minimum_required(VERSION 2.8.0)
project(test_bond)

find_package(catkin REQUIRED)
catkin_package()

find_package(catkin REQUIRED bondcpp genmsg rostest)

include_directories(SYSTEM ${bondcpp_INCLUDE_DIRS})

# create the service
add_service_files(DIRECTORY srv FILES TestBond.srv)
generate_messages()

# add more tests
catkin_add_gtest(exercise_bond test/exercise_bond.cpp)
target_link_libraries(exercise_bond ${catkin_LIBRARIES})
add_dependencies(exercise_bond ${PROJECT_NAME}_gencpp)

add_rostest(test/test_cpp.launch)
add_rostest(test/test_python.launch)

catkin_add_gtest(test_callbacks_cpp test/test_callbacks_cpp.cpp)
target_link_libraries(test_callbacks_cpp ${catkin_LIBRARIES})
add_dependencies(test_callbacks_cpp ${PROJECT_NAME}_gencpp)

add_rostest(test/test_callbacks_cpp.launch)
add_rostest(test/test_callbacks_py.launch)
