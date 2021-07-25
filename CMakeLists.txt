cmake_minimum_required(VERSION 3.19)
project(MPC_Controller)

set(CMAKE_CXX_STANDARD 14)

find_package(kindr)
include_directories(${kindr_INCLUDE_DIRS})

find_package(control_toolbox)
include_directories(${control_toolbox_INCLUDE_DIRS})

find_package(Threads)

#add_executable(MPC_Example src/MPC_example.cpp)

#add_executable(classical_Direct_Multiple_Shooting src/classical_Direct_Multiple_Shooting.cpp)
#
add_executable(MPC_Controller src/MPC_Controller.cpp)

target_link_libraries (MPC_Controller ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(MPC_Controller "/usr/local/lib/libmatio.so")
target_link_libraries(MPC_Controller "/usr/local/lib/libhpipm.a")