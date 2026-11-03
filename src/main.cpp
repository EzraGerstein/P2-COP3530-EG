#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <queue>
#include <thread>
#include <chrono>
#include <optional>
#include <limits>
#include <atomic>
#include <memory>
#include <algorithm>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
// I added this after windows.h because for some reason it gives an error when it's above.
#include "graph.h"
#include "min_heap.h"
#include "dijkstra.h"

// changed to gredcell to compatability reasons
struct Node {
    int node_id;
    double x_coord;
    double y_coord;
    double terrain_weight;
    std::vector<int> neighbors;
};

// States for visual search display
enum NodeState {
    UNVISITED,
    FRONTIER,   // Yellow: Unfinished paths actively being evaluated (Search Frontier)
    VISITED,    // Blue: Explored nodes
    PATH        // Red: Final completed shortest path
};

// Available pathfinding algorithms
enum Algorithm {
    DIJKSTRA,
    ASTAR
};

// Element stored in custom min-heap
struct VisualHeapElement {
    int node_id;
    double priority;
};

// Custom Binary Min-Heap built from scratch (no std::priority_queue)
class VisualMinHeap {
    std::vector<VisualHeapElement> heap;
    std::vector<int> node_to_heap_idx;

    void siftUp(int idx) {
        while (idx > 0) {
            int parent = (idx - 1) / 2;
            if (heap[idx].priority < heap[parent].priority) {
                swapElements(idx, parent);
                idx = parent;
            } else {
                break;
            }
        }
    }

    void siftDown(int idx) {
        int size = static_cast<int>(heap.size());
        while (2 * idx + 1 < size) {
            int left = 2 * idx + 1;
            int right = 2 * idx + 2;
            int smallest = left;

            if (right < size && heap[right].priority < heap[left].priority) {
                smallest = right;
            }

            if (heap[smallest].priority < heap[idx].priority) {
                swapElements(idx, smallest);
                idx = smallest;
            } else {
                break;
            }
        }
    }

    void swapElements(int i, int j) {
        std::swap(heap[i], heap[j]);
        node_to_heap_idx[heap[i].node_id] = i;
        node_to_heap_idx[heap[j].node_id] = j;
    }

public:
    explicit VisualMinHeap(size_t total_nodes) {
        node_to_heap_idx.assign(total_nodes, -1);
    }

    bool empty() const {
        return heap.empty();
    }

    void pushOrDecreaseKey(int node_id, double priority) {
        if (node_to_heap_idx[node_id] == -1) {
            heap.push_back({node_id, priority});
            int idx = static_cast<int>(heap.size()) - 1;
            node_to_heap_idx[node_id] = idx;
            siftUp(idx);
        } else {
            int idx = node_to_heap_idx[node_id];
            if (priority < heap[idx].priority) {
                heap[idx].priority = priority;
                siftUp(idx);
            }
        }
    }

    VisualHeapElement extractMin() {
        VisualHeapElement minElem = heap[0];
        swapElements(0, static_cast<int>(heap.size()) - 1);
        node_to_heap_idx[minElem.node_id] = -1;
        heap.pop_back();

        if (!heap.empty()) {
            siftDown(0);
        }
        return minElem;
    }
};

// Euclidean distance heuristic for A* Search
inline double calculateHeuristic(const Node& a, const Node& b) {
    double dx = a.x_coord - b.x_coord;
    double dy = a.y_coord - b.y_coord;
    return std::sqrt(dx * dx + dy * dy);
}

class TrailQuestSystem {
public:
    int width = 0;
    int height = 0;
    std::vector<Node> nodes;
    std::unique_ptr<std::atomic<NodeState>[]> node_states;
    std::string console_log;   // print summary only after app closes

    std::atomic<bool> stop_requested{false};
    std::atomic<bool> is_searching{false};
    std::atomic<int> nodes_explored{0};
    std::atomic<double> final_path_distance{0.0};
    std::atomic<Algorithm> current_algo{ASTAR};

    TrailQuestSystem(const std::string& grid_filename) {
        loadGridFromFile(grid_filename);
        if (width != 100 || height != 100) {
            generateDefaultSquareGrid(100, 100);
        }
    }

    void resetStates() {
        if (node_states) {
            for (int i = 0; i < width * height; ++i) {
                node_states[i] = UNVISITED;
            }
        }
        nodes_explored = 0;
        final_path_distance = 0.0;
    }

    // Generate a 1:1 true square 100x100 campus map (10,000 nodes) with realistic terrain
    void generateDefaultSquareGrid(int w = 100, int h = 100) {
        width = w;
        height = h;
        nodes.clear();
        nodes.resize(width * height);
        node_states.reset(new std::atomic<NodeState>[width * height]);
        resetStates();

        for (int r = 0; r < height; ++r) {
            for (int c = 0; c < width; ++c) {
                int id = r * width + c;
                nodes[id].node_id = id;
                nodes[id].x_coord = c * 2.5;
                nodes[id].y_coord = r * 2.5;

                // Base terrain: Grass (Light Green)
                double weight = 2.5;

                // Lake Alice (Water: 99.0) in the center
                double dx = (c - 50.0) / 20.0;
                double dy = (r - 50.0) / 14.0;
                double lake_dist = dx * dx + dy * dy;
                if (lake_dist <= 1.0) {
                    weight = 99.0;
                }

                // Smaller wetland pond to the northeast
                double px = (c - 76.0) / 8.0;
                double py = (r - 28.0) / 6.0;
                if (px * px + py * py <= 1.0) {
                    weight = 99.0;
                }

                // Campus Buildings (Dark Red: 60.0)
                if ((r >= 10 && r <= 20 && c >= 28 && c <= 42) ||
                    (r >= 12 && r <= 24 && c >= 62 && c <= 76) ||
                    (r >= 74 && r <= 86 && c >= 18 && c <= 32) ||
                    (r >= 70 && r <= 82 && c >= 64 && c <= 78)) {
                    weight = 60.0;
                }

                // Perimeter Roads (Gray: 8.0)
                if ((c == 15 || c == 85) && r >= 5 && r <= 95) weight = 8.0;
                if ((r == 15 || r == 85) && c >= 5 && c <= 95) weight = 8.0;

                // Nature Trails & Footpaths (Green: 1.0)
                // Lake Alice shoreline walking loop
                if (lake_dist > 1.0 && lake_dist <= 1.35) {
                    weight = 1.0;
                }
                // Diagonal connector trails from start and goal heading to the lake loop
                if (std::abs(r - c) <= 1 && (r <= 36 || r >= 64)) {
                    weight = 1.0;
                }
                // North-South connecting trail bypasses
                if (c == 28 && r >= 36 && r <= 64) weight = 1.0;
                if (c == 72 && r >= 36 && r <= 64) weight = 1.0;

                nodes[id].terrain_weight = weight;
            }
        }

        // Wire up 4-way Adjacency List connections across the 2D grid
        for (int y_idx = 0; y_idx < height; ++y_idx) {
            for (int x_idx = 0; x_idx < width; ++x_idx) {
                int idx = y_idx * width + x_idx;
                if (x_idx > 0) nodes[idx].neighbors.push_back(idx - 1);           // Left
                if (x_idx < width - 1) nodes[idx].neighbors.push_back(idx + 1);       // Right
                if (y_idx > 0) nodes[idx].neighbors.push_back(idx - width);       // Up
                if (y_idx < height - 1) nodes[idx].neighbors.push_back(idx + width);  // Down
            }
        }

        // Persist the 100x100 grid to disk
        auto saveTo = [&](const std::string& path) {
            std::ofstream out(path);
            if (out.is_open()) {
                out << width << " " << height << "\n";
                for (const auto& n : nodes) {
                    out << n.node_id << " " << n.x_coord << " " << n.y_coord << " " << n.terrain_weight << "\n";
                }
            }
        };
        saveTo("grid.txt");
        saveTo("../grid.txt");

        console_log += "Generated and saved 100x100 square campus grid (10,000 nodes).\n";
    }

    // Load the 100,000+ node terrain grid exported by Python
    void loadGridFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            file.open("../" + filename);
        }
        if (!file.is_open()) {
            std::cerr << "Grid file not found. Generating default 100x100 square grid...\n";
            generateDefaultSquareGrid(100, 100);
            return;
        }

        file >> width >> height;
        if (width != 100 || height != 100) {
            file.close();
            generateDefaultSquareGrid(100, 100);
            return;
        }

        nodes.resize(width * height);
        node_states.reset(new std::atomic<NodeState>[width * height]); // each element starts at zero for unvisited
        resetStates();

        int id;
        double x, y, weight;
        while (file >> id >> x >> y >> weight) {
            if (id >= 0 && id < static_cast<int>(nodes.size())) {
                nodes[id].node_id = id;
                nodes[id].x_coord = x;
                nodes[id].y_coord = y;
                nodes[id].terrain_weight = weight;
            }
        }
        file.close();

        // Wire up 4-way Adjacency List connections across the 2D grid
        for (int y_idx = 0; y_idx < height; ++y_idx) {
            for (int x_idx = 0; x_idx < width; ++x_idx) {
                int idx = y_idx * width + x_idx;
                if (x_idx > 0) nodes[idx].neighbors.push_back(idx - 1);           // Left
                if (x_idx < width - 1) nodes[idx].neighbors.push_back(idx + 1);       // Right
                if (y_idx > 0) nodes[idx].neighbors.push_back(idx - width);       // Up
                if (y_idx < height - 1) nodes[idx].neighbors.push_back(idx + width);  // Down
            }
        }

        console_log += "Successfully loaded and wired " + std::to_string(nodes.size()) + " nodes from " + filename + ".\n";
    }

    // Pathfinding simulation supporting both Dijkstra and A* with custom MinHeap
    void runSearchSimulation(Algorithm algo, int start_idx, int end_idx) {
        if (nodes.empty()) return;

        stop_requested = false;
        is_searching = true;
        current_algo = algo;
        nodes_explored = 0;
        final_path_distance = 0.0;

        VisualMinHeap pq(nodes.size());

        std::vector<double> g_cost(nodes.size(), std::numeric_limits<double>::infinity());
        std::vector<bool> visited(nodes.size(), false);
        std::vector<int> came_from(nodes.size(), -1);

        g_cost[start_idx] = 0.0;
        double start_priority = (algo == ASTAR) ? calculateHeuristic(nodes[start_idx], nodes[end_idx]) : 0.0;
        pq.pushOrDecreaseKey(start_idx, start_priority);
        node_states[start_idx] = FRONTIER;

        // Pacing: A* visits far fewer nodes, so use smaller batches and a steady pause
        // so you can clearly watch its targeted search progression.
        const int BATCH_SIZE = (algo == ASTAR) ? 3 : 20;
        const int BATCH_PAUSE_MS = (algo == ASTAR) ? 28 : 15;
        int steps_in_batch = 0;

        while (!pq.empty() && !stop_requested) {
            VisualHeapElement current_elem = pq.extractMin();
            int current = current_elem.node_id;

            if (visited[current]) continue;
            visited[current] = true;
            nodes_explored++;

            if (current == end_idx) {
                std::vector<int> path_nodes;
                int curr = end_idx;
                while (curr != -1) {
                    path_nodes.push_back(curr);
                    curr = came_from[curr];
                }
                final_path_distance = g_cost[end_idx];
                std::string algo_str = (algo == ASTAR) ? "A* Search" : "Dijkstra";
                console_log += algo_str + " finished: Path found! Visited " + std::to_string(nodes_explored.load()) +
                               " nodes. Total cost: " + std::to_string(final_path_distance.load()) + "\n";

                // Smoothly animate the final shortest path lighting up from start to the flag!
                std::reverse(path_nodes.begin(), path_nodes.end());
                for (int p_node : path_nodes) {
                    if (stop_requested) break;
                    node_states[p_node] = PATH;
                    std::this_thread::sleep_for(std::chrono::milliseconds(18));
                }
                break;
            }

            node_states[current] = VISITED;

            for (int neighbor : nodes[current].neighbors) {
                if (visited[neighbor]) continue;

                // Skip impassable or extremely high cost barriers (e.g., Water cost 99.0)
                if (nodes[neighbor].terrain_weight >= 99.0) continue;

                double weight = nodes[neighbor].terrain_weight;
                double tentative_gCost = g_cost[current] + weight;

                if (tentative_gCost < g_cost[neighbor]) {
                    g_cost[neighbor] = tentative_gCost;
                    came_from[neighbor] = current;
                    node_states[neighbor] = FRONTIER;

                    double priority = tentative_gCost;
                    if (algo == ASTAR) {
                        priority += calculateHeuristic(nodes[neighbor], nodes[end_idx]);
                    }
                    pq.pushOrDecreaseKey(neighbor, priority);
                }
            }

            steps_in_batch++;
            if (steps_in_batch >= BATCH_SIZE) {
                steps_in_batch = 0;
                std::this_thread::sleep_for(std::chrono::milliseconds(BATCH_PAUSE_MS));
            }
        }

        is_searching = false;
    }
};

int main() {
    TrailQuestSystem system("grid.txt");

    if (system.nodes.empty()) {
        std::cerr << "Grid failed to load. Ensure grid.txt is in the same directory.\n";
        return -1;
    }

    sf::RenderWindow window(sf::VideoMode({800, 800}), "TrailQuest - [1] Dijkstra  [2] A*  [R] Reset");
    window.setFramerateLimit(60);

    int start_node = 20 * system.width + 20;
    int end_node = (system.height - 20) * system.width + (system.width - 20);

#ifdef _WIN32
    timeBeginPeriod(1); // added this so the animation paces out better on Windows
#endif

    std::thread search_thread;

    // Helper lambda to safely cancel and restart search simulation
    auto launchSearch = [&](Algorithm algo) {
        system.stop_requested = true;
        if (search_thread.joinable()) {
            search_thread.join();
        }
        system.resetStates();
        search_thread = std::thread(&TrailQuestSystem::runSearchSimulation, &system, algo, start_node, end_node);
    };

    // Default start with A* search
    launchSearch(ASTAR);

    int frame_counter = 0;

    // added my search if something is not working lemme know
    TerrainGraph terrain_graph;
    terrain_graph.load_grid("grid.txt");

    std::string search_summary;
    std::thread dijkstra_thread([&]() {
        Search_Result result = find_shortest_path(terrain_graph, start_node, end_node);

        if (result.path_exist) {
            search_summary = "dijkstra: path " + std::to_string(result.path_found.size()) + " nodes, cost " +
                std::to_string(result.path_total_cost) + ", explored " + std::to_string(result.Nodes_used) +
                " nodes in " + std::to_string(result.search_time) + " ms\n";
        } else {
            search_summary = "dijkstra: no path found\n";
        }
    });

    while (window.isOpen()) {
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }

            // Handle window resizing dynamically while keeping 1:1 pixel coordinates
            if (const auto* resized = event->getIf<sf::Event::Resized>()) {
                window.setView(sf::View(sf::FloatRect({0.f, 0.f}, sf::Vector2f(resized->size))));
            }

            // Keyboard controls for switching algorithms on the fly
            if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
                if (keyPressed->code == sf::Keyboard::Key::Num1 || keyPressed->code == sf::Keyboard::Key::D) {
                    launchSearch(DIJKSTRA);
                } else if (keyPressed->code == sf::Keyboard::Key::Num2 || keyPressed->code == sf::Keyboard::Key::A) {
                    launchSearch(ASTAR);
                } else if (keyPressed->code == sf::Keyboard::Key::R || keyPressed->code == sf::Keyboard::Key::Space) {
                    launchSearch(system.current_algo.load());
                }
            }
        }

        // Periodically update window title with live statistics
        frame_counter++;
        if (frame_counter % 10 == 0) {
            std::string algo_name = (system.current_algo == ASTAR) ? "A* Search" : "Dijkstra";
            std::string status = system.is_searching ? "Exploring..." : "Complete";
            std::string title = "TrailQuest | [1] Dijkstra  [2] A*  [R] Reset | Active: " + algo_name +
                                " (" + status + ") | Visited: " + std::to_string(system.nodes_explored.load());
            if (!system.is_searching && system.final_path_distance > 0.0) {
                title += " | Distance: " + std::to_string(static_cast<int>(system.final_path_distance.load()));
            }
            window.setTitle(title);
        }

        window.clear(sf::Color(15, 15, 20));

        float winWidth = static_cast<float>(window.getSize().x);
        float winHeight = static_cast<float>(window.getSize().y);

        // Strict 1:1 aspect ratio: cells are always true squares
        float cellSize = std::min(winWidth / system.width, winHeight / system.height);
        if (cellSize < 1.0f) cellSize = 1.0f;

        // Center the map in the window
        float offsetX = (winWidth - cellSize * system.width) / 2.0f;
        float offsetY = (winHeight - cellSize * system.height) / 2.0f;

        sf::RectangleShape rect(sf::Vector2f(cellSize, cellSize));

        for (int y = 0; y < system.height; ++y) {
            for (int x = 0; x < system.width; ++x) {
                int idx = y * system.width + x;
                rect.setPosition({offsetX + x * cellSize, offsetY + y * cellSize});

                // Render based on search states first, then fallback to terrain colors
                switch (system.node_states[idx]) {
                    case FRONTIER:
                        rect.setFillColor(sf::Color::Yellow);         // Yellow: Search Frontier
                        break;
                    case VISITED:
                        // MATTHEW: just a heads up, I like the blue and the navy, but I think
                        // they're a little too similar. The sweep is a bit hard to see on water.
                        // Either add a legend somewhere that tells us what each color means, or
                        // just change the sweep color from navy to something else.
                        rect.setFillColor(sf::Color(100, 150, 255));    // Blue: Explored nodes
                        break;
                    case PATH:
                        rect.setFillColor(sf::Color::Red);            // Red: Final Shortest Path
                        break;
                    case UNVISITED: {
                        double w = system.nodes[idx].terrain_weight;
                        if (w == 1.0)
                            rect.setFillColor(sf::Color(0, 200, 0));    // Green: Trails (Easy)
                        else if (w == 2.5)
                            rect.setFillColor(sf::Color(120, 180, 80)); // Light Green: Grass
                        else if (w == 8.0)
                            rect.setFillColor(sf::Color(100, 100, 100));// Gray: Roads
                        else if (w == 60.0)
                            rect.setFillColor(sf::Color(180, 50, 50));  // Dark Red: Buildings
                        else
                            rect.setFillColor(sf::Color(30, 30, 100));  // Deep Blue/Cyan: Water (99.0)
                        break;
                    }
                }
                window.draw(rect);
            }
        }

        // --- Draw Start Marker (Bright Green Pin) ---
        float startX = offsetX + (start_node % system.width + 0.5f) * cellSize;
        float startY = offsetY + (start_node / system.width + 0.5f) * cellSize;
        float startRadius = std::max(4.0f, cellSize * 0.9f);

        sf::CircleShape startMarker(startRadius);
        startMarker.setOrigin({startRadius, startRadius});
        startMarker.setPosition({startX, startY});
        startMarker.setFillColor(sf::Color(40, 230, 40));
        startMarker.setOutlineColor(sf::Color::White);
        startMarker.setOutlineThickness(std::max(1.5f, cellSize * 0.2f));
        window.draw(startMarker);

        // --- Draw Endpoint Destination Flag ---
        float endX = offsetX + (end_node % system.width + 0.5f) * cellSize;
        float endY = offsetY + (end_node / system.width + 0.5f) * cellSize;

        // Glowing target ring around endpoint
        float ringRadius = std::max(6.0f, cellSize * 1.3f);
        sf::CircleShape goalRing(ringRadius);
        goalRing.setOrigin({ringRadius, ringRadius});
        goalRing.setPosition({endX, endY});
        goalRing.setFillColor(sf::Color::Transparent);
        goalRing.setOutlineColor(sf::Color(255, 50, 50, 220));
        goalRing.setOutlineThickness(std::max(2.0f, cellSize * 0.25f));
        window.draw(goalRing);

        // Flagpole
        float poleW = std::max(2.5f, cellSize * 0.25f);
        float poleH = std::max(18.0f, cellSize * 2.8f);
        sf::RectangleShape pole(sf::Vector2f(poleW, poleH));
        pole.setFillColor(sf::Color(240, 240, 240));
        pole.setOutlineColor(sf::Color(30, 30, 30));
        pole.setOutlineThickness(1.0f);
        pole.setPosition({endX - poleW / 2.0f, endY - poleH + ringRadius * 0.3f});
        window.draw(pole);

        // Gold finial ball atop flagpole
        float ballRadius = std::max(2.5f, poleW * 1.2f);
        sf::CircleShape finial(ballRadius);
        finial.setOrigin({ballRadius, ballRadius});
        finial.setPosition({pole.getPosition().x + poleW / 2.0f, pole.getPosition().y});
        finial.setFillColor(sf::Color(255, 215, 0));
        window.draw(finial);

        // Triangular red finish flag waving outward
        float flagW = std::max(14.0f, cellSize * 2.2f);
        float flagH = std::max(10.0f, cellSize * 1.4f);
        sf::ConvexShape flag(3);
        flag.setPoint(0, sf::Vector2f(pole.getPosition().x + poleW, pole.getPosition().y));
        flag.setPoint(1, sf::Vector2f(pole.getPosition().x + poleW + flagW, pole.getPosition().y + flagH * 0.5f));
        flag.setPoint(2, sf::Vector2f(pole.getPosition().x + poleW, pole.getPosition().y + flagH));
        flag.setFillColor(sf::Color(240, 30, 30));
        flag.setOutlineColor(sf::Color(140, 0, 0));
        flag.setOutlineThickness(1.0f);
        window.draw(flag);

        window.display();
    }

    system.stop_requested = true;
    if (search_thread.joinable()) {
        search_thread.join();
    }

    if (dijkstra_thread.joinable()) {
        dijkstra_thread.join();
    }

#ifdef _WIN32
    timeEndPeriod(1);
#endif

    // The console should not print anything until the application has closed.
    std::cout << system.console_log << search_summary;

    return 0;
}