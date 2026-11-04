#pragma once


#include <vector>
#include <cmath>
#include <chrono>
#include <limits>
#include <algorithm>
#include <iostream>
#include <utility>

struct Node {
    int id;
    double x;
    double y;
    double terrainWeight;
    std::vector<std::pair<int, double>> neighbors;
};

struct BenchmarkMetrics {
    double executionTimeMS;
    int nodesVisited;
    double totalPathDistance;
    int memoryUsedKB;
    std::vector<int> path;
};

struct HeapElement {
    int nodeID;
    double fCost;
};

class MinHeap {
    std::vector<HeapElement> heap;
    std::vector<int> NodeToHeapIdx;

    void siftUp(int idx);
    void siftDown(int idx);
    void swapElements(int i, int j);

public:
    explicit MinHeap(size_t total_nodes);
    bool empty() const;
    void pushOrDecreaseKey(int nodeID, double fCost);
    HeapElement extractMin();
    int getMemoryFootprintBytes() const;
};

double calculateEuclideanHeuristic(const Node& a, const Node& b, double min_terrain_weight = 1.0);

BenchmarkMetrics runAStar(const std::vector<Node>& graph, int start_id, int goal_id);

std::vector<Node> generateTerrainGrid(int rows, int cols);
