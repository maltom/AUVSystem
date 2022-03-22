project(AUVSystem)

set(CMAKE_CXX_STANDARD 17)

cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

include_directories(external . /opt/ros/noetic/include /opt/ros/noetic/lib)

set(NAME Commons)

option(NOLQR "Working without LQR or model estimation" OFF)
option(SIMULATION "Working in simulation mode" OFF)

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

if(NOLQR)
    add_definitions(-DNOLQR)
endif()
if(SIMULATION)
    add_definitions(-DSIMULATION)
endif()

#add_subdirectory(AdditionalCameraProcessor)
add_subdirectory(Debug)
add_subdirectory(EmergencyRestartProtocol)
add_subdirectory(HealthCheck)
add_subdirectory(MainLogic)
# add_subdirectory(PositioningSLAM)
add_subdirectory(TCPNode)
# add_subdirectory(TrajectoryPlanner)
add_subdirectory(ThrusterRegulator)
add_subdirectory(UDPNode)
add_subdirectory(external)

add_library(${NAME} ${SOURCES} ${HEADERS})
set_target_properties( ${NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${NAME}/" )

target_link_libraries(${NAME} Eigen3::Eigen
    -I/opt/ros/noetic/include -L/opt/ros/noetic/lib
    -lroscpp -lrostime -lrosconsole -lroscpp_serialization
)
