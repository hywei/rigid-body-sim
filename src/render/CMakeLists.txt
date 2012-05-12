include_directories(
        ${PROJECT_SOURCE_DIR}/src/common
        ${PROJECT_SOURCE_DIR}/src/rigid
        ${PROJECT_SOURCE_DIR}/src/collision
        ${PROJECT_SOURCE_DIR}/src/simulation
        )

file(GLOB HEADERS *.hh)
file(GLOB SOURCES *.cc)

add_library(render STATIC ${HEADERS} ${SOURCES})
