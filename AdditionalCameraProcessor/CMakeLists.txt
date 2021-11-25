set(NODENAME AdditionalCameraProcessor)

include_directories(../)

set(MODULES 
    Commons
    Externals
)

set(SOURCES
    src/main.cpp
    src/AdditionalCameraProcessor.cpp
    src/CameraReceiver.cpp
)

set(HEADERS
    src/AdditionalCameraProcessor.h
    src/CameraReceiver.h
)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs rospy)
find_package( OpenCV REQUIRED )

include_directories(include ${catkin_INCLUDE_DIRS} )
include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(${NODENAME} ${SOURCES} ${HEADERS})
set_target_properties( ${NODENAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${NODENAME}/")

target_link_libraries(${NODENAME} ${MODULES} ${catkin_LIBRARIES} ${OpenCV_LIBS})
