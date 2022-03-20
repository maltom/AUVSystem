set(NAME Externals)

set(SOURCES
    EigenQP/EigenQP.cpp
    jsonxx/jsonxx.cc
)

set(HEADERS
    EigenQP/EigenQP.h
    jsonxx/jsonxx.h
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(${NAME} ${SOURCES} ${HEADERS})

target_link_libraries(${NAME} Eigen3::Eigen)
