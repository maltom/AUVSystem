set(NODENAME TCPNode)

include_directories(../)

set(MODULES 
    Commons
    Externals
)

set(SOURCES
    src/main.cpp
    src/MessageProcessing.cpp
    src/TCPNode.cpp
    src/TCPClient.cpp
)

set(HEADERS
    src/MessageProcessing.h
    src/TCPNode.h
    src/TCPClient.h
)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs rospy)

include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(${NODENAME} ${SOURCES} ${HEADERS})
set_target_properties( ${NODENAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${NODENAME}/" )

target_link_libraries(${NODENAME} ${MODULES} ${catkin_LIBRARIES})