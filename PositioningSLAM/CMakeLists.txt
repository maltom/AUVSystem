set(NODENAME PositioningSLAM)

include_directories(../)

set(MODULES 
    Commons
    Externals
)

set(SOURCES
    src/main.cpp
    src/PositioningSLAM.cpp
)

set(HEADERS
    src/PositioningSLAM.h
)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs rospy)
find_package(ct_core REQUIRED NO_MODULE)


include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(${NODENAME} ${SOURCES} ${HEADERS})
set_target_properties( ${NODENAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${NODENAME}/" )

target_link_libraries(${NODENAME} ${MODULES} ${catkin_LIBRARIES})