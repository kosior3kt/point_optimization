add_library(my_math_lib 
   src/lib/lib.cpp
   src/lib/lib.h
   src/lib/math_structures.hpp
   )

target_include_directories(my_math_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
