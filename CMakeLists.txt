project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(SOURCES
    utility.cc
    main.cc
    )

add_executable(ExtendedKF ${SOURCES})
