# Group 66 - TrailQuest

Hello, we made TrailQuest, and we're testing Dijkstra's and A* search algorithms on a map based on realistic terrain around the UF campus and Gainesville. We used map info for nodes that was sourced from OpenStreetMap.

## How to run

To run the program, you need `grid.txt` in the same folder as the exe. In order to run this, you will need to have the DLLs next to the exe.

You can either double-click the exe, or open a command prompt in the folder and run:

    main.exe

If you want to build it yourself, you'll need CMake and MinGW:

    cmake -B build
    cmake --build build
