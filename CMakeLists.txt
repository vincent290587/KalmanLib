cmake_minimum_required (VERSION 3.5.1)

project (KalmanTester)

set(CMAKE_COLOR_MAKEFILE                        OFF)

# On inclut notre bibliotheque dans le processus de CMake
# add_subdirectory(source/routes)

# The version number.
set (Tutorial_VERSION_MAJOR 1)
set (Tutorial_VERSION_MINOR 0)

add_definitions(-DTDD)
add_definitions(-D__HEAP_SIZE=150000)
add_definitions(-D_USE_MATH_DEFINES)

set(CMAKE_C_FLAGS  "-fno-builtin --short-enums -nostdlib -MMD -nostdlib -ffunction-sections -fdata-sections -fno-strict-aliasing")
set(CMAKE_CXX_FLAGS  "-g -Og -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti -fpermissive")


# add the binary tree to the search path for include files

include_directories(
    ./
    ./TDD
    source
	source/kalman
    )

# Cree des variables avec les fichiers a compiler
set(SRCS
	source/kalman/kalman_ext.cpp
	source/kalman/UDMatrix.cpp
    )
    
aux_source_directory(./TDD      TDD)
    
 
# add the executable
add_executable(KalmanTester ${SRCS} ${TDD} ${HEADERS})
