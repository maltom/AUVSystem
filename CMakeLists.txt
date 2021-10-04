project(AUVSystem)

set(CMAKE_CXX_STANDARD 17)

cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

add_subdirectory(MainLogic)
add_subdirectory(TrajectoryPlanner)
add_subdirectory(ThrusterRegulator)
add_subdirectory(UDPNode)