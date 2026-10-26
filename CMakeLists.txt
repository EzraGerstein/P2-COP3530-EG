cmake_minimum_required(VERSION 3.20)
project(untitled5)

set(CMAKE_CXX_STANDARD 17)

# SFML 3 uses "Graphics", "Window", "System" (case-sensitive components)
find_package(SFML 3.0 COMPONENTS Graphics Window System REQUIRED)

add_executable(untitled5 main.cpp)

# Link SFML 3 libraries (note the capitalized module names)
target_link_libraries(untitled5 sfml-Graphics sfml-Window sfml-System)