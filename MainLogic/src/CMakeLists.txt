set(NODENAME StateMachine)

include_directories(../external)

set(SOURCES
    StateMachine/StateMachine.cpp
    StateMachine/StateStack.cpp
    StateMachine/StateHandler.cpp
)

set(HEADERS
    src/StateMachine.h
    StateMachine/StateStack.h
    StateMachine/StateHandler.h
)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs rospy)

include_directories(include ${catkin_INCLUDE_DIRS} )
