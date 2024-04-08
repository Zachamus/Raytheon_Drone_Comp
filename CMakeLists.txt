cmake_minimum_required(VERSION 3.27)
project(Raytheon_Drone_Comp)

set(CMAKE_CXX_STANDARD 14)

include_directories(Raytheon_Capstone)

add_executable(Raytheon_Drone_Comp
        Raytheon_Capstone/Move.cpp
        Raytheon_Capstone/Move.h
        Raytheon_Capstone/OpenCV.cpp
        Raytheon_Capstone/OpenCV.h
        Raytheon_Capstone/Raytheon_Capstone.cpp)
