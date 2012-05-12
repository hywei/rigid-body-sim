message(${PROJECT_SOURCE_DIR})
include_directories(
        ${PROJECT_SOURCE_DIR}/src/common
        ${PROJECT_SOURCE_DIR}/src/collision
        ${PROJECT_SOURCE_DIR}/src/render
        ${PROJECt_SOURCE_DIR}/src/simulation
        )

file(GLOB HEADERS *.hh)
file(GLOB SOURCES *.cc)

add_library(rigid STATIC ${HEADERS} ${SOURCES})
