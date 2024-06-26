cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(Raytheon_Capstone)

add_executable(Raytheon_Capstone
    Raytheon_Capstone.cpp
    Header.h
        SearchAlgo.cpp
        SearchAlgo.h
        OpenCV.h
)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/home/raytheon/Downloads/GeographicLib-2.3/cmake")
find_package(GeographicLib REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)
pkg_check_modules(GPIOD REQUIRED libgpiod)
include_directories(${GPIOD_INCLUDE_DIRS})

include_directories(${GeographicLib_INCLUDE_DIRS})
target_link_libraries(Raytheon_Capstone ${GeographicLib_LIBRARIES})
target_link_libraries( Raytheon_Capstone ${OpenCV_LIBS} )
target_link_libraries(Raytheon_Capstone ${GPIOD_LIBRARIES})
include_directories(${LIBCAMERA_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
target_link_libraries(Raytheon_Capstone -llccv ${OpenCV_LIBS})

target_link_libraries(Raytheon_Capstone
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(offboard PRIVATE -Wall -Wextra)
else()
    add_compile_options(offboard PRIVATE -WX -W2)
endif()

if(MINGW OR MSVC)
    # For M_PI
    add_target_definitions(offboard "-D_USE_MATH_DEFINES")
endif()
