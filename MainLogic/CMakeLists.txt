set(NODENAME MainLogic)

include_directories(../)

set(MODULES 
    StateMachine
    Commons
    Externals
)

set(SOURCES
    src/main.cpp
    src/MainLogic.cpp
)

set(HEADERS
    src/MainLogic.h
)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs rospy)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_subdirectory(src)

add_executable(${NODENAME} ${SOURCES} ${HEADERS})
set_target_properties( ${NODENAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${NODENAME}/" )

target_link_libraries(${NODENAME} ${catkin_LIBRARIES} ${MODULES} ${EIGEN3_LIBRARIES})
