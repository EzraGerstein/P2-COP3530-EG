cmake_minimum_required(VERSION 3.20)
project(untitled5)

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

add_executable(untitled5 main.cpp)

# Link SFML 3 libraries. The namespaced targets already carry the include dirs.
target_link_libraries(untitled5 PRIVATE SFML::Graphics SFML::Window SFML::System)

# timeBeginPeriod and timeEndPeriod are in the Windows multimedia library.
if(WIN32)
    target_link_libraries(untitled5 PRIVATE winmm)
endif()

# MATTHEW: just a heads up, if you want to make it portable I can help
# if you need, but it'll make grading easier. There are two ways to do it.
# Either add this line after the executable, then rebuild so the EXE
# carries its own runtime:
#   target_link_options(untitled5 PRIVATE -static-libgcc -static-libstdc++ -static)
# or just copy these three DLLs next to the EXE, together with grid.txt:
#   libgcc_s_seh-1.dll, libstdc++-6.dll, libwinpthread-1.dll
# I think that should be fine. If you have any questions, let me know.