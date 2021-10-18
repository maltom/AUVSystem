set(NODENAME ThrusterRegulator)

include_directories(../external)
include_directories(../)

set(SOURCES
    src/main.cpp
    src/EigenQP.cpp
    src/ThrusterRegulator.cpp
    src/VehiclePhysicalModel.cpp
    ../external/jsonxx/jsonxx.cc
    ../jsonCommonFunctions.cpp
    ../NodeBase.cpp
    ../ROSEnums.cpp
)

set(HEADERS
    src/EigenQP.h
    src/ThrusterRegulator.h
    src/VehiclePhysicalModel.h
    ../external/jsonxx/jsonxx.h
    ../jsonCommonFunctions.h
    ../NodeBase.h
    ../ROSEnums.h
)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs rospy)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(${NODENAME} ${SOURCES} ${HEADERS})
set_target_properties( ${NODENAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY build )

target_link_libraries(${NODENAME} ${catkin_LIBRARIES} Eigen3::Eigen)