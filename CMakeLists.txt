cmake_minimum_required(VERSION 3.22)
project(Project2)

set(CMAKE_CXX_STANDARD 17)

# Try to include a sfml-static inside the repo so it's not hard coded.
# Hopefully that works. If it fails, use the one on your computer.
# I don't know where it's located, but just set it to be your SFML
# and that should be it.
if(NOT SFML_DIR AND EXISTS "${CMAKE_SOURCE_DIR}/sfml-static/lib/cmake/SFML/SFMLConfig.cmake")
    set(SFML_DIR "${CMAKE_SOURCE_DIR}/sfml-static/lib/cmake/SFML")
    set(SFML_STATIC_LIBRARIES ON)
endif()

# SFML 3 uses "Graphics", "Window", "System" (case-sensitive components)
find_package(SFML 3.0 COMPONENTS Graphics Window System REQUIRED)

include_directories(src)

# Ensure grid.txt is copied to the binary directory so the executable finds it when run from CLion
configure_file("${CMAKE_SOURCE_DIR}/grid.txt" "${CMAKE_CURRENT_BINARY_DIR}/grid.txt" COPYONLY)

add_executable(Project2 main.cpp graph.cpp min_heap.cpp dijkstra.cpp)

# Link SFML 3 libraries. The namespaced targets already carry the include dirs.
target_link_libraries(Project2 PRIVATE SFML::Graphics SFML::Window SFML::System)

# Commented out for CLion
# include(CTest)
# include(Catch)
# catch_discover_tests(Tests)

# timeBeginPeriod and timeEndPeriod are in the Windows multimedia library.
if(WIN32)
    target_link_libraries(Project2 PRIVATE winmm)
endif()

# MATTHEW: just a heads up, if you want to make it portable I can help
# if you need, but it'll make grading easier. There are two ways to do it.
# Either add this line after the executable, then rebuild so the EXE
# carries its own runtime:

# target_link_options(Project2 PRIVATE -static-libgcc -static-libstdc++ -static)
# or just copy these three DLLs next to the EXE, together with grid.txt:
#   libgcc_s_seh-1.dll, libstdc++-6.dll, libwinpthread-1.dll
# I think that should be fine. If you have any questions, let me know.

# --- Benchmark CLI Executable (A* and Heap) ---
add_executable(astar_benchmark src/main.cpp src/AStar.cpp)
target_include_directories(astar_benchmark PRIVATE ${CMAKE_SOURCE_DIR}/src)

# --- Unit Tests Executable (Catch2) ---
add_executable(run_tests test/test.cpp test/catch/catch_amalgamated.cpp src/AStar.cpp)
target_include_directories(run_tests PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/test/catch)


