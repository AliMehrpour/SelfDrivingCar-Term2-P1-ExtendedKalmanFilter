project(ExtendedKalmanFilter)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
    ./main.cpp
    ./data_source.cpp
    ./fusion_ekf.cpp
    ./kalman_filter.cpp
    ./tools.cpp
    )

add_executable(ExtendedKF ${sources})
	