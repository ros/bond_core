cmake_minimum_required(VERSION 2.8.3)
project(bondcpp)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED bond roscpp smclib)

include_directories(SYSTEM ${BOOST_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} uuid
  CATKIN_DEPENDS bond roscpp smclib
  DEPENDS Boost
)

add_library(${PROJECT_NAME}
  src/timeout.cpp
  src/bond.cpp
  src/BondSM_sm.cpp
)
target_link_libraries(${PROJECT_NAME} uuid ${catkin_LIBRARIES} ${BOOST_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif()

# install stuff
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
