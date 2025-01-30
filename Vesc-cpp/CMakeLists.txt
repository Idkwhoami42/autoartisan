cmake_minimum_required(VERSION 3.30)
project(motor_test)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_VERBOSE_MAKEFILE ON)

include_directories("C:/Boost/include/boost-1_87")
#link_directories("C:/Users/lilis/Downloads/boost_1_87_0/boost_1_87_0/boost")
link_directories("C:/Boost/lib")

add_executable(motor_test main.cpp
        motor.h
        commands.h
        motor.cpp
        crc.h
)

target_link_libraries(motor_test boost_system)
# target_link_libraries(motor_test libboost_system-vc143-mt-gd-x64-1_87 ws2_32)
