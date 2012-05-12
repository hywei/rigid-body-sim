include_directories(
        ${PROJECT_SOURCE_DIR}/src/common
        ${PROJECT_SOURCE_DIR}/src/rigid
        ${PROJECT_SOURCE_DIR}/src/render
        ${PROJECT_SOURCE_DIR}/src/simulation
        )

file(GLOB HEADERS *.hh)
file(GLOB SOURCES *.cc)

add_library(collision STATIC ${HEADERS} ${SOURCES})
