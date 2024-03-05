cmake_minimum_required(VERSION 3.12)
project(YourProjectName)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

add_executable(dupa main.cpp)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS}) #trzeba includowac to gowno, zeby ceres i eigen zczepily sie dupamu
target_link_libraries(dupa PRIVATE ${CERES_LIBRARIES})

find_package(glog REQUIRED)
target_link_libraries(dupa PRIVATE glog::glog)

find_package(Eigen3 REQUIRED)
target_link_libraries(dupa PRIVATE Eigen3::Eigen)

# Disable precompiled headers
set(ENABLE_PRECOMPILED_HEADERS OFF) #this is made so that it won't have problems when doing some spicy testing 

# Add the option to the compiler
target_compile_options(dupa PRIVATE -DENABLE_PRECOMPILED_HEADERS=${ENABLE_PRECOMPILED_HEADERS})

