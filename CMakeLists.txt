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
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(Debug)
add_subdirectory(Dummy)
add_subdirectory(HealthCheck)
add_subdirectory(MainLogic)
add_subdirectory(PositioningSLAM)
add_subdirectory(TCPNode)
add_subdirectory(TrajectoryPlanner)
add_subdirectory(ThrusterRegulator)
add_subdirectory(UDPNode)
add_subdirectory(external)

add_library(${NAME} ${SOURCES} ${HEADERS})
set_target_properties( ${NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${NAME}/" )

target_link_libraries(${NAME} Eigen3::Eigen )
