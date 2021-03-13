set(NODENAME StateMachine)

include_directories(../external)

set(SOURCES
    src/main.cpp
    src/StateMachine.cpp
    ../external/jsonxx/jsonxx.cc
)

set(HEADERS
    src/StateMachine.h
    ../external/jsonxx/jsonxx.h
)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs rospy)

include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(${NODENAME} ${SOURCES} ${HEADERS})
set_target_properties( ${NODENAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY build )

target_link_libraries(${NODENAME} ${catkin_LIBRARIES})