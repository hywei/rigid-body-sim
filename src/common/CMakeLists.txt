include_directories(
        ${PROJECT_SOURCE_DIR}/src/rigid
        ${PROJECT_SOURCE_DIR}/src/collision
        ${PROJECT_SOURCE_DIR}/src/render
        ${PROJECt_SOURCE_DIR}/src/simulation
        )

file(GLOB HEADERS *.hh)
file(GLOB SOURCES *.cc *.inl)

add_library(common STATIC ${HEADERS} ${SOURCES})

