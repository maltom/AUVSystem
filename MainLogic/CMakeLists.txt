set(NODENAME MainLogic)

include_directories(../external)
include_directories(../)

set(MODULES 
    StateMachine
)

set(SOURCES
    src/main.cpp
    src/MainLogic.cpp
    ../external/jsonxx/jsonxx.cc
    ../jsonCommonFunctions.cpp
    ../NodeBase.cpp
    ../ROSEnums.cpp
)

set(HEADERS
    src/MainLogic.h
    ../external/jsonxx/jsonxx.h
    ../jsonCommonFunctions.h
    ../NodeBase.h
    ../ROSEnums.h
)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs rospy)

include_directories(include ${catkin_INCLUDE_DIRS} )

add_subdirectory(src)

add_executable(${NODENAME} ${SOURCES} ${HEADERS})
set_target_properties( ${NODENAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY build )

target_link_libraries(${NODENAME} ${catkin_LIBRARIES} ${MODULES})