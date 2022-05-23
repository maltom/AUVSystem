set(NODENAME ThrusterRegulator)

include_directories(../)
include_directories(../external/EigenQP/)

set(MODULES 
    Commons
    Externals
)

set(SOURCES
    src/main.cpp
    src/LQRRegulator.cpp
    src/ThrusterRegulator.cpp
    src/VehiclePhysicalModel.cpp
)

set(HEADERS
    src/LQRRegulator.h
    src/ThrusterRegulator.h
    src/VehiclePhysicalModel.h
)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs rospy)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(LAPACK)
find_package(ct_optcon)

include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(${NODENAME} ${SOURCES} ${HEADERS})
set_target_properties( ${NODENAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${NODENAME}/" )

target_link_libraries(${NODENAME} ${catkin_LIBRARIES} ${MODULES} Eigen3::Eigen ct_optcon lapack)