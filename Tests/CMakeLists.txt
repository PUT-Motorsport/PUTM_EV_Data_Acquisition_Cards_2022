CMAKE_MINIMUM_REQUIRED(VERSION 3.4)

project(Kalman_tests)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic -Werror")
set(CMAKE_CXX_FLAGS "-O3")

add_executable(Kalman_tests kalman_tests.cpp)