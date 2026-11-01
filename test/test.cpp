#include "catch_amalgamated.hpp"
#include "AStar.h"

TEST_CASE("MinHeap: Basic insert and extract-min ordering", "[minheap]") {
    MinHeap heap(10);
    REQUIRE(heap.empty() == true);

    heap.pushOrDecreaseKey(1, 15.0);
    heap.pushOrDecreaseKey(2, 5.0);
    heap.pushOrDecreaseKey(3, 10.0);
    heap.pushOrDecreaseKey(4, 2.5);

    REQUIRE_FALSE(heap.empty());

    HeapElement e1 = heap.extractMin();
    REQUIRE(e1.nodeID == 4);
    REQUIRE(e1.fCost == 2.5);

    HeapElement e2 = heap.extractMin();
    REQUIRE(e2.nodeID == 2);
    REQUIRE(e2.fCost == 5.0);

    HeapElement e3 = heap.extractMin();
    REQUIRE(e3.nodeID == 3);
    REQUIRE(e3.fCost == 10.0);

    HeapElement e4 = heap.extractMin();
    REQUIRE(e4.nodeID == 1);
    REQUIRE(e4.fCost == 15.0);

    REQUIRE(heap.empty() == true);
}

TEST_CASE("MinHeap: pushOrDecreaseKey priority reduction", "[minheap]") {
    MinHeap heap(5);

    heap.pushOrDecreaseKey(0, 50.0);
    heap.pushOrDecreaseKey(1, 30.0);
    heap.pushOrDecreaseKey(2, 40.0);

    heap.pushOrDecreaseKey(0, 10.0);

    HeapElement top = heap.extractMin();
    REQUIRE(top.nodeID == 0);
    REQUIRE(top.fCost == 10.0);
}

TEST_CASE("Heuristic: Euclidean distance calculations and admissibility", "[heuristic]") {
    Node a{0, 0.0, 0.0, 1.0, {}};
    Node b{1, 3.0, 4.0, 1.0, {}};
    Node c{2, 0.0, 0.0, 1.0, {}};

    double dist = calculateEuclideanHeuristic(a, b);
    REQUIRE(dist == Catch::Approx(5.0));

    REQUIRE(calculateEuclideanHeuristic(a, c) == Catch::Approx(0.0));
}

TEST_CASE("A* Search: Start node is the goal node", "[astar]") {
    std::vector<Node> graph = generateTerrainGrid(3, 3);
    BenchmarkMetrics result = runAStar(graph, 4, 4);

    REQUIRE(result.path.size() == 1);
    REQUIRE(result.path[0] == 4);
    REQUIRE(result.totalPathDistance == Catch::Approx(0.0));
}

TEST_CASE("A* Search: Finds optimal shortest path on 3x3 grid", "[astar]") {
    std::vector<Node> graph = generateTerrainGrid(3, 3);
    BenchmarkMetrics result = runAStar(graph, 0, 8);

    REQUIRE_FALSE(result.path.empty());
    REQUIRE(result.path.front() == 0);
    REQUIRE(result.path.back() == 8);
    REQUIRE(result.nodesVisited > 0);
}

TEST_CASE("A* Search: Reroutes around high-penalty obstacle", "[astar]") {
    std::vector<Node> graph = generateTerrainGrid(3, 3);
    graph[4].terrainWeight = 500.0;

    BenchmarkMetrics result = runAStar(graph, 0, 8);

    REQUIRE_FALSE(result.path.empty());
    for (int node_id : result.path) {
        REQUIRE(node_id != 4);
    }
}

TEST_CASE("A* Search: Handles disconnected / unreachable node gracefully", "[astar]") {
    std::vector<Node> graph(3);
    graph[0] = {0, 0.0, 0.0, 1.0, {{1, 1.0}}};
    graph[1] = {1, 1.0, 0.0, 1.0, {{0, 1.0}}};
    graph[2] = {2, 5.0, 5.0, 1.0, {}};

    BenchmarkMetrics result = runAStar(graph, 0, 2);

    REQUIRE(result.path.empty());
}

