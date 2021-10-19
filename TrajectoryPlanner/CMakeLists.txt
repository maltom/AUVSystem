set(NODENAME TrajectoryPlanner)

include_directories(../)
include_directories(/usr/local/include/)


set(MODULES 
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
find_package(ct_core REQUIRED NO_MODULE)


include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(${NODENAME} ${SOURCES} ${HEADERS})
set_target_properties( ${NODENAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY build )

target_link_libraries(${NODENAME} ${MODULES} ${catkin_LIBRARIES})