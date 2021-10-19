set(NODENAME TrajectoryPlanner)

include_directories(../)

set(MODULES 
    StateMachine
    Commons
    Externals
)

set(SOURCES
    src/main.cpp
    src/TrajectoryPlanner.cpp
)

set(HEADERS
    src/TrajectoryPlanner.h
)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs rospy)

include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(${NODENAME} ${SOURCES} ${HEADERS})
set_target_properties( ${NODENAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY build )

target_link_libraries(${NODENAME} ${MODULES} ${catkin_LIBRARIES})