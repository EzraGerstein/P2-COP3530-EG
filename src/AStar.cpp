#include <AStar.h>

using namespace std;

struct Node {
    int id;
    double x;
    double y;
    double terrainWeight;
    vector<pair<int, double>> neighbors;
};

struct BenchmarkMetrics {
    double executionTimeMS;
    int nodesVisited;
    double totalPathDistance;
    int memoryUsedKB;
    vector<int> path;
};

struct HeapElement {
    int nodeID;
    double fCost;
};

class MinHeap {
    vector<HeapElement> heap;
    vector<int> NodeToHeapIdx;

    void siftUp(int idx) {
        while (idx > 0) {
            int parent = (idx - 1) / 2;
            if (heap[idx].fCost < heap[parent].fCost) {
                swapElements(idx, parent);
                idx = parent;
            } else {
                break;
            }
        }
    }

    void siftDown(int idx) {
        int size = heap.size();
        while (2 * idx + 1 < size) {
            int left = 2 * idx + 1;
            int right = 2 * idx + 2;
            int smallest = left;

            if (right < size && heap[right].fCost < heap[left].fCost) {
                smallest = right;
            }

            if (heap[smallest].fCost < heap[idx].fCost) {
                swapElements(idx, smallest);
                idx = smallest;
            } else {
                break;
            }
        }
    }

    void swapElements(int i, int j) {
        std::swap(heap[i], heap[j]);
        NodeToHeapIdx[heap[i].nodeID] = i;
        NodeToHeapIdx[heap[j].nodeID] = j;
    }

public:
    MinHeap(size_t total_nodes) {
        NodeToHeapIdx.assign(total_nodes, -1);
    }

    bool empty() const {
        return heap.empty();
    }

    void pushOrDecreaseKey(int nodeID, double fCost) {
        if (NodeToHeapIdx[nodeID] == -1) {
            heap.push_back({nodeID, fCost});
            int idx = heap.size() - 1;
            NodeToHeapIdx[nodeID] = idx;
            siftUp(idx);
        } else {
            int idx = NodeToHeapIdx[nodeID];
            if (fCost < heap[idx].fCost) {
                heap[idx].fCost = fCost;
                siftUp(idx);
            }
        }
    }

    HeapElement extractMin() {
        HeapElement minElem = heap[0];
        swapElements(0, heap.size() - 1);
        NodeToHeapIdx[minElem.nodeID] = -1;
        heap.pop_back();

        if (!heap.empty()) {
            siftDown(0);
        }
        return minElem;
    }

    int getMemoryFootprintBytes() const {
        return (heap.capacity() * sizeof(HeapElement)) + 
               (NodeToHeapIdx.capacity() * sizeof(int));
    }
};


inline double calculateEuclideanHeuristic(Node& a, Node& b, double min_terrain_weight = 1.0) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy) * min_terrain_weight;
}

BenchmarkMetrics runAStar(vector<Node>& graph, int start_id, int goal_id) {
    BenchmarkMetrics metrics = {0.0, 0, 0.0, 0, {}};
    const size_t n = graph.size();

    if (start_id < 0 || start_id >= n ||
        goal_id < 0 || goal_id >= n) {
        cerr << "Invalid start or destination node ID.\n";
        return metrics;
    }

    auto start_time = chrono::high_resolution_clock::now();

    double INF = numeric_limits<double>::infinity();
    std::vector<double> g_score(n, INF);
    std::vector<double> f_score(n, INF);
    std::vector<int> parent(n, -1);
    std::vector<bool> closed(n, false);

    MinHeap open_set(n);

    g_score[start_id] = 0.0;
    f_score[start_id] = calculateEuclideanHeuristic(graph[start_id], graph[goal_id]);
    open_set.pushOrDecreaseKey(start_id, f_score[start_id]);

    bool found = false;

    while (!open_set.empty()) {
        const HeapElement current = open_set.extractMin();
        int u = current.nodeID;

        if (closed[u]) continue;
        closed[u] = true;
        metrics.nodesVisited++;
        
        if (u == goal_id) {
            found = true;
            break;
        }

        for (const auto& edge : graph[u].neighbors) {
            const int v = edge.first;
            const double base_dist = edge.second;

            if (closed[v]) continue;

            const double travel_cost = base_dist * graph[v].terrainWeight;
            const double tentative_g = g_score[u] + travel_cost;

            if (tentative_g < g_score[v]) {
                parent[v] = u;
                g_score[v] = tentative_g;
                f_score[v] = tentative_g + calculateEuclideanHeuristic(graph[v], graph[goal_id]);
                open_set.pushOrDecreaseKey(v, f_score[v]);
            }
        }
    }

    auto end_time = chrono::high_resolution_clock::now();
    metrics.executionTimeMS = chrono::duration<double, milli>(end_time - start_time).count();

    size_t vectors_memory = (sizeof(double) * 2 + sizeof(int) + sizeof(bool)) * n;
    metrics.memoryUsedKB = (vectors_memory + open_set.getMemoryFootprintBytes()) / 1024;

    if (!found) {
        std::cout << "No route found between selected nodes.\n";
        return metrics;
    }

    int curr = goal_id;
    while (curr != -1) {
        metrics.path.push_back(curr);
        if (curr == start_id) break;
        curr = parent[curr];
    }
    std::reverse(metrics.path.begin(), metrics.path.end());
    metrics.totalPathDistance = g_score[goal_id];

    return metrics;
}

inline std::vector<Node> generateTerrainGrid(int rows, int cols) {
    const int total = rows * cols;
    std::vector<Node> graph(total);

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            int id = r * cols + c;
            graph[id].id = id;
            graph[id].x = static_cast<double>(c);
            graph[id].y = static_cast<double>(r);
            graph[id].terrainWeight = 1.0;

            for (int dr = -1; dr <= 1; ++dr) {
                for (int dc = -1; dc <= 1; ++dc) {
                    if (dr == 0 && dc == 0) continue;
                    const int nr = r + dr;
                    const int nc = c + dc;
                    if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
                        int nid = nr * cols + nc;
                        double dist = (dr != 0 && dc != 0) ? std::sqrt(2.0) : 1.0;
                        graph[id].neighbors.emplace_back(nid, dist);
                    }
                }
            }
        }
    }
    return graph;
}