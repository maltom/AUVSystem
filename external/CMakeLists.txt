set(NAME Externals)

set(SOURCES
    jsonxx/jsonxx.cc
)

set(HEADERS
    jsonxx/jsonxx.h
)

add_library(${NAME} ${SOURCES} ${HEADERS})
set_target_properties( ${NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY build )