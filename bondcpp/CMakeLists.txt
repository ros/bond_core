cmake_minimum_required(VERSION 2.8.0)
project(bondcpp)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED roscpp)
catkin_package(LIBRARIES ${PROJECT_NAME}
               DEPENDS Boost roscpp
)

include_directories(SYSTEM ${BOOST_INCLUDE_DIRS})
include_directories(include)

add_library(${PROJECT_NAME}
  src/timeout.cpp
  src/bond.cpp
  src/BondSM_sm.cpp
)
target_link_libraries(${PROJECT_NAME} ${BOOST_LIBRARIES})

# install stuff
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
# vrabaud: not sure why this one is not in a folder
install(FILES include/BondSM_sm.h
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
