#include <AStar.cpp>

int main() {
    // fake data
    const int ROWS = 320;
    const int COLS = 320;
    vector<Node> graph = generateTerrainGrid(ROWS, COLS);

    // fake obstacles
    for (int r = 100; r <= 200; ++r) {
        for (int c = 100; c <= 200; ++c) {
            graph[r * COLS + c].terrainWeight = 25.0;
        }
    }

    // start top left, end bottom right
    int start_node = 0;
    int target_node = (ROWS - 1) * COLS + (COLS - 1);

    const BenchmarkMetrics result = runAStar(graph, start_node, target_node);

    // temporary output
    cout << "Metric                       | A* Search Result  \n";
    cout << "-------------------------------------------------\n";
    cout << "Execution Time (ms)          | " << result.executionTimeMS << " ms\n";
    cout << "Nodes Visited / Expanded     | " << result.nodesVisited << "\n";
    cout << "Total Path Cost / Distance   | " << result.totalPathDistance << " m\n";
    cout << "Memory Used (KB)             | " << result.memoryUsedKB << " KB\n";
    cout << "Path Length (Vertices)       | " << result.path.size() << " nodes\n";
    cout << "-------------------------------------------------\n";
}
#include <iostream>

using namespace std;

int main() {
    cout << "hello world!";
}
