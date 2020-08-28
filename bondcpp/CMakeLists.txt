cmake_minimum_required(VERSION 3.0.2)
project(bondcpp)

find_package(Boost REQUIRED date_time thread)
find_package(catkin REQUIRED bond cmake_modules roscpp smclib)
find_package(UUID REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${UUID_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS bond roscpp smclib
  DEPENDS Boost UUID
)

add_library(${PROJECT_NAME}
  src/timeout.cpp
  src/bond.cpp
  src/BondSM_sm.cpp
)
target_link_libraries(${PROJECT_NAME} ${UUID_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if(catkin_EXPORTED_TARGETS)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif()

# install stuff
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
