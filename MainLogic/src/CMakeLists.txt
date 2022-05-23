set(NAME StateMachine)


set(SOURCES
    StateMachine/LogicCommonData.cpp
    StateMachine/StateStack.cpp
    StateMachine/States/StateBase.cpp
    StateMachine/States/StateDiveToDepth.cpp
    StateMachine/States/StateEnums.cpp
    StateMachine/States/StateFindGate.cpp
    StateMachine/States/StateGoLittleTowardsGate.cpp
    StateMachine/States/StateIdle.cpp
    StateMachine/States/StateMission.cpp
    StateMachine/States/StateTask1.cpp
    StateMachine/States/StateTask2.cpp
)

set(HEADERS
    StateMachine/LogicCommonData.h
    StateMachine/StateStack.h
    StateMachine/States/StateBase.h
    StateMachine/States/StateDiveToDepth.h
    StateMachine/States/StateEnums.h
    StateMachine/States/StateFindGate.h
    StateMachine/States/StateGoLittleTowardsGate.h
    StateMachine/States/StateIdle.h
    StateMachine/States/StateMission.h
    StateMachine/States/StateTask1.h
    StateMachine/States/StateTask2.h
)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs rospy)

include_directories(include ${catkin_INCLUDE_DIRS} .)

add_library(${NAME} ${SOURCES} ${HEADERS})
# NODENAME is not a mistake here
set_target_properties( ${NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${NODENAME}/" )