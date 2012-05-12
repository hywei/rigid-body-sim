include_directories(
        ${PROJECT_SOURCE_DIR}/src/common
        ${PROJECT_SOURCE_DIR}/src/rigid
        ${PROJECT_SOURCE_DIR}/src/collision
        ${PROJECT_SOURCE_DIR}/src/render
        )

file(GLOB HEADERS *.hh)
file(GLOB SOURCES *.cc)

add_library(simulation STATIC ${HEADERS} ${SOURCES})
