set(NAME StateMachine)


set(SOURCES
    StateMachine/StateMachine.cpp
    StateMachine/StateStack.cpp
    StateMachine/States/StateBase.cpp
    StateMachine/States/StateIdle.cpp
    StateMachine/States/StateEnums.cpp
    StateMachine/States/SubStateBase.cpp
)

set(HEADERS
    StateMachine/StateMachine.h
    StateMachine/StateStack.h
    StateMachine/States/StateBase.h
    StateMachine/States/StateIdle.h
    StateMachine/States/StateEnums.h
    StateMachine/States/SubStateBase.h
)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs rospy)

include_directories(include ${catkin_INCLUDE_DIRS} )

add_library(${NAME} ${SOURCES} ${HEADERS})
# NODENAME is not a mistake here
set_target_properties( ${NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${NODENAME}/" )