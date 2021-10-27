set(NODENAME Dummy)

include_directories(../)

set(SOURCES
    Dummy.cpp
)

add_executable(${NODENAME} ${SOURCES})
set_target_properties( ${NODENAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${NODENAME}/")
