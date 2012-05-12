cmake_minimum_required(VERSION 2.8)

if(WIN32)
        set(CMAKE_FIND_LIBRARY_PREFIXES "")
        set(CMAKE_FIND_LIBRARY_SUFFIXES ".lib")
else()
        set(CMAKE_FIND_LIBRARY_PREFIXES "lib")
        set(CMAKE_FIND_LIBRARY_SUFFIXES ".so")
endif()


if(WIN32)		
else()
	set( Boost_INCLUDE_DIR "/usr/include/")
endif()

SET(CMAKE_CXX_FLAGS "-O3 -finline-functions -Winline -Wall -pedantic")

project(RigidBody-Simulation)


message("${BOOST_INCLUDE_DIR}")
message("${CMAKE_BUILD_TYPE}")
message("${CMAKE_CXX_COMPILER}")

if(WIN32)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
else()
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
endif()
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

add_subdirectory(src/common)
add_subdirectory(src/rigid)
add_subdirectory(src/collision)
add_subdirectory(src/render)
add_subdirectory(src/simulation)
add_subdirectory(src/mainwindow)
        
       
