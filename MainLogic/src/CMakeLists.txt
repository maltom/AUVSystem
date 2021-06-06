set(NODENAME StateMachine)


set(SOURCES
    StateMachine/StateMachine.cpp
    StateMachine/StateStack.cpp
    StateMachine/States/StateBase.cpp
    StateMachine/States/StateIdle.cpp
    StateMachine/States/StateEnums.cpp
)

set(HEADERS
    StateMachine/StateMachine.h
    StateMachine/StateStack.h
    StateMachine/States/StateBase.h
    StateMachine/States/StateIdle.h
    StateMachine/States/StateEnums.h
)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs rospy)

include_directories(include ${catkin_INCLUDE_DIRS} )

add_library(${NODENAME} ${SOURCES} ${HEADERS})
set_target_properties( ${NODENAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY build )