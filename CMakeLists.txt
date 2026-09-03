cmake_minimum_required(VERSION 3.14)
project(ins_uav CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

add_library(ins_ekf
  src/InsEkf.cpp
)
target_include_directories(ins_ekf PUBLIC include)

add_executable(sim_flight examples/sim_flight.cpp)
target_link_libraries(sim_flight PRIVATE ins_ekf)

enable_testing()
add_executable(test_ins_ekf tests/test_ins_ekf.cpp)
target_link_libraries(test_ins_ekf PRIVATE ins_ekf)
target_include_directories(test_ins_ekf PRIVATE examples)
add_test(NAME test_ins_ekf COMMAND test_ins_ekf)
