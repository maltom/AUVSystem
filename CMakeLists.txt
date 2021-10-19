project(AUVSystem)

set(CMAKE_CXX_STANDARD 17)

cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

include_directories(external .)

set(NAME Commons)

set(SOURCES
    CommonEnums.cpp
    jsonCommonFunctions.cpp
    NodeBase.cpp
    ROSEnums.cpp
)

set(HEADERS
    CommonEnums.h
    jsonCommonFunctions.h
    NodeBase.h
    ROSEnums.h
)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs rospy)

include_directories(include ${catkin_INCLUDE_DIRS})

add_subdirectory(MainLogic)
add_subdirectory(TrajectoryPlanner)
add_subdirectory(ThrusterRegulator)
add_subdirectory(UDPNode)
add_subdirectory(external)

add_library(${NAME} ${SOURCES} ${HEADERS})
set_target_properties( ${NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY . )

target_link_libraries(${NAME} ${catkin_LIBRARIES})


