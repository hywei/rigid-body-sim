include_directories(
        ${PROJECT_SOURCE_DIR}/src/common
        ${PROJECT_SOURCE_DIR}/src/rigid
        ${PROJECT_SOURCE_DIR}/src/collision
        ${PROJECT_SOURCE_DIR}/src/render
        ${PROJECT_SOURCE_DIR}/src/simulation
        )


file(GLOB HEADERS *.hh)
file(GLOB SOURCES *.cc)

add_executable(rigid-sim ${HEADERS} ${SOURCES})

set (DEPENDENCIES simulation render collision rigid common)

add_dependencies(common rigid collision render simulation)

target_link_libraries(rigid-sim GL GLU glut ${DEPENDENCIES})
                        
