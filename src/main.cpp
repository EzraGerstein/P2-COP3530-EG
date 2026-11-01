#include "AStar.h"
#include <iostream>

int main() {
    const int ROWS = 320;
    const int COLS = 320;
    std::cout << "=================================================\n";
    std::cout << " TrailQuest: A* Pathfinding Benchmark Runner     \n";
    std::cout << "=================================================\n";
    std::cout << "Generating " << (ROWS * COLS) << "-node terrain grid (" << ROWS << "x" << COLS << ")...\n";
    
    std::vector<Node> graph = generateTerrainGrid(ROWS, COLS);

    for (int r = 100; r <= 200; ++r) {
        for (int c = 100; c <= 200; ++c) {
            graph[r * COLS + c].terrainWeight = 25.0;
        }
    }

    int default_start = 0;
    int default_target = (ROWS - 1) * COLS + (COLS - 1);

    int start_node = default_start;
    int target_node = default_target;

    std::cout << "\nTotal nodes in graph: " << graph.size() << " (IDs 0 to " << graph.size() - 1 << ")\n";
    std::cout << "Enter Start Node ID [default " << default_start << "]: ";
    if (!(std::cin >> start_node) || start_node < 0 || static_cast<size_t>(start_node) >= graph.size()) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        start_node = default_start;
        std::cout << "Using default start node: " << start_node << "\n";
    }

    std::cout << "Enter Target Node ID [default " << default_target << "]: ";
    if (!(std::cin >> target_node) || target_node < 0 || static_cast<size_t>(target_node) >= graph.size()) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        target_node = default_target;
        std::cout << "Using default target node: " << target_node << "\n";
    }

    std::cout << "\nRunning A* Search from node " << start_node << " to node " << target_node << "...\n";
    const BenchmarkMetrics result = runAStar(graph, start_node, target_node);

    std::cout << "\n-------------------------------------------------\n";
    std::cout << "Metric                       | A* Search Result  \n";
    std::cout << "-------------------------------------------------\n";
    std::cout << "Execution Time (ms)          | " << result.executionTimeMS << " ms\n";
    std::cout << "Nodes Visited / Expanded     | " << result.nodesVisited << "\n";
    std::cout << "Total Path Cost / Distance   | " << result.totalPathDistance << "\n";
    std::cout << "Memory Used (KB)             | " << result.memoryUsedKB << " KB\n";
    std::cout << "Path Length (Vertices)       | " << result.path.size() << " nodes\n";
    std::cout << "-------------------------------------------------\n";

    return 0;
}
