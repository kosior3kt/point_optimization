Description:
So this is gonna be one day a algebra library for matching points based on the given data into specified shape/size

Licence:
GLWTS (Good Luck With That Shieeet)


In order to build this beauty, you need to get conan2 and CMake
just run: 
conan install . --output-folder=build2 --build=missing
((conan may need you tu run some profile detection, refer to the online documentation for that))
cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build .

voila, that should be it




