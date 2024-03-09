cmake_minimum_required(VERSION 3.15)
project(point_optimization CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

#looking for all the components and including their directories if needed
find_package(glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS}) #trzeba includowac to gowno, zeby ceres i eigen zczepily sie dupamu

#pretty self-explainatory
add_subdirectory(lib)
add_subdirectory(example)

#link the shit of this baby 
target_link_libraries(my_math_lib PUBLIC ${CERES_LIBRARIES})
target_link_libraries(my_math_lib PUBLIC glog::glog)
target_link_libraries(my_math_lib PUBLIC Eigen3::Eigen)

#this section is probably redundant, but why not
# Disable precompiled headers
set(ENABLE_PRECOMPILED_HEADERS OFF) #this is made so that it won't have problems when doing some spicy testing 

# Add the option to the compiler
target_compile_options(my_math_lib PRIVATE -DENABLE_PRECOMPILED_HEADERS=${ENABLE_PRECOMPILED_HEADERS})

