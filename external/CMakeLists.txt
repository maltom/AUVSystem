set(NAME Externals)

set(SOURCES
    EigenQP/EigenQP.cpp
    jsonxx/jsonxx.cc
)

set(HEADERS
    EigenQP/EigenQP.h
    jsonxx/jsonxx.h
)

add_library(${NAME} ${SOURCES} ${HEADERS})