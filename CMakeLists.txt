<<<<<<< Updated upstream
cmake_minimum_required(VERSION 3.22)
project(Project2)
=======
cmake_minimum_required(VERSION 3.20)
project(UFTrailQuest)
>>>>>>> Stashed changes

set(CMAKE_CXX_STANDARD 14)

#compile flags to match Gradescope test environment
set(GCC_COVERAGE_COMPILE_FLAGS "-Wall -Werror") # remove -Wall if you don't want as many warnings treated as errors
# gradescope does use -Werror, so if you remove it here you may run into issues when trying to submit
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )

##pull in desired version of catch through cmake automatically, make it available
Include(FetchContent)
FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.5.2 # if you run into issues, try updating your editor or downgrading to a lower version of catch v3
)
FetchContent_MakeAvailable(Catch2)

<<<<<<< Updated upstream
include_directories(src)

add_executable(Main
        src/main.cpp
        src/AStar.cpp
        src/AStar.h # your main file
        # add your own header files below - should be automatically added in CLion
        # example (can also separate with newlines):
        # src/AVL.h src/AVL.cpp
        )
        
# These tests can use the Catch2-provided main
add_executable(Tests
        test/test.cpp # your test file
        # add your own header files below - should be automatically added in CLion
        # example (can also separate with newlines):
        # src/AVL.h src/AVL.cpp
        )
        
target_link_libraries(Tests PRIVATE Catch2::Catch2WithMain) #link catch to test.cpp file
# the name here must match that of your testing executable (the one that has test.cpp)

# comment everything below out if you are using CLion
include(CTest)
include(Catch)
catch_discover_tests(Tests) # must be named the same as your test executable
=======
add_executable(UFTrailQuest main.cpp)

# Link SFML 3 libraries. The namespaced targets already carry the include dirs.
target_link_libraries(UFTrailQuest PRIVATE SFML::Graphics SFML::Window SFML::System)

# timeBeginPeriod and timeEndPeriod are in the Windows multimedia library.
if(WIN32)
    target_link_libraries(UFTrailQuest PRIVATE winmm)
endif()

# MATTHEW: just a heads up, if you want to make it portable I can help
# if you need, but it'll make grading easier. There are two ways to do it.
# Either add this line after the executable, then rebuild so the EXE
# carries its own runtime:
#   target_link_options(UFTrailQuest PRIVATE -static-libgcc -static-libstdc++ -static)
# or just copy these three DLLs next to the EXE, together with grid.txt:
#   libgcc_s_seh-1.dll, libstdc++-6.dll, libwinpthread-1.dll
# I think that should be fine. If you have any questions, let me know.

# --- Benchmark CLI Executable (A* and Heap) ---
add_executable(astar_benchmark src/main.cpp src/AStar.cpp)
target_include_directories(astar_benchmark PRIVATE ${CMAKE_SOURCE_DIR}/src)

# --- Unit Tests Executable (Catch2) ---
add_executable(run_tests test/test.cpp test/catch/catch_amalgamated.cpp src/AStar.cpp)
target_include_directories(run_tests PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/test/catch)
>>>>>>> Stashed changes
