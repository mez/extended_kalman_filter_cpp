project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x -Wall)

set(SOURCES
    utility.cc
    ekf.cc
    fusion_ekf.cc
    main.cc
    )

add_executable(ExtendedKF ${SOURCES})
