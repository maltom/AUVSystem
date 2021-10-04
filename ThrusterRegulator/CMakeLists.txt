set(NODENAME ThrusterRegulator)

include_directories(../external)
include_directories(../)

set(SOURCES
    src/main.cpp
    src/ThrusterRegulator.cpp
    ../external/jsonxx/jsonxx.cc
    ../jsonCommonFunctions.cpp
    ../NodeBase.cpp
    ../ROSEnums.cpp
)

set(HEADERS
    src/ThrusterRegulator.h
    ../external/jsonxx/jsonxx.h
    ../jsonCommonFunctions.h
    ../NodeBase.h
    ../ROSEnums.h
)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs rospy)

include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(${NODENAME} ${SOURCES} ${HEADERS})
set_target_properties( ${NODENAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY build )

target_link_libraries(${NODENAME} ${catkin_LIBRARIES})