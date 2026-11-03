#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
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

// Stores the most recent benchmark run metrics for in-screen side-by-side display
struct AlgorithmStats {
    std::atomic<bool> has_run{false};
    std::atomic<bool> is_running{false};
    std::atomic<double> search_time_ms{0.0};
    std::atomic<int> nodes_explored{0};
    std::atomic<double> total_cost{0.0};
    std::atomic<int> path_length{0};
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

    AlgorithmStats dijkstra_stats;
    AlgorithmStats astar_stats;

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

    void resetAllStats() {
        resetStates();
        dijkstra_stats.has_run = false;
        dijkstra_stats.is_running = false;
        dijkstra_stats.search_time_ms = 0.0;
        dijkstra_stats.nodes_explored = 0;
        dijkstra_stats.total_cost = 0.0;
        dijkstra_stats.path_length = 0;

        astar_stats.has_run = false;
        astar_stats.is_running = false;
        astar_stats.search_time_ms = 0.0;
        astar_stats.nodes_explored = 0;
        astar_stats.total_cost = 0.0;
        astar_stats.path_length = 0;
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

        if (algo == DIJKSTRA) dijkstra_stats.is_running = true;
        if (algo == ASTAR) astar_stats.is_running = true;

        auto t_start = std::chrono::high_resolution_clock::now();

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

            if (algo == DIJKSTRA) dijkstra_stats.nodes_explored = nodes_explored.load();
            if (algo == ASTAR) astar_stats.nodes_explored = nodes_explored.load();

            if (current == end_idx) {
                auto t_end = std::chrono::high_resolution_clock::now();
                double elapsed = std::chrono::duration<double, std::milli>(t_end - t_start).count();

                std::vector<int> path_nodes;
                int curr = end_idx;
                while (curr != -1) {
                    path_nodes.push_back(curr);
                    curr = came_from[curr];
                }
                final_path_distance = g_cost[end_idx];

                if (algo == DIJKSTRA) {
                    dijkstra_stats.search_time_ms = elapsed;
                    dijkstra_stats.nodes_explored = nodes_explored.load();
                    dijkstra_stats.total_cost = final_path_distance.load();
                    dijkstra_stats.path_length = static_cast<int>(path_nodes.size());
                    dijkstra_stats.has_run = true;
                    dijkstra_stats.is_running = false;
                } else {
                    astar_stats.search_time_ms = elapsed;
                    astar_stats.nodes_explored = nodes_explored.load();
                    astar_stats.total_cost = final_path_distance.load();
                    astar_stats.path_length = static_cast<int>(path_nodes.size());
                    astar_stats.has_run = true;
                    astar_stats.is_running = false;
                }

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

        if (algo == DIJKSTRA) dijkstra_stats.is_running = false;
        if (algo == ASTAR) astar_stats.is_running = false;
        is_searching = false;
    }
};

int main() {
    TrailQuestSystem system("grid.txt");

    if (system.nodes.empty()) {
        std::cerr << "Grid failed to load. Ensure grid.txt is in the same directory.\n";
        return -1;
    }

    // 1200 x 800 window: Map takes left 800x800, Analytics dashboard takes right 400x800
    sf::RenderWindow window(sf::VideoMode({1200, 800}), "Project 2");
    window.setFramerateLimit(60);

    // Font loading with multi-path fallback
    sf::Font font;
    bool font_loaded = false;
    const std::vector<std::string> font_candidates = {
        "assets/arial.ttf",
        "../assets/arial.ttf",
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/segoeui.ttf"
    };
    for (const auto& path : font_candidates) {
        if (font.openFromFile(path)) {
            font_loaded = true;
            break;
        }
    }
    if (!font_loaded) {
        std::cerr << "Warning: Could not load TTF font. Dashboard text will not render.\n";
    }

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

    // Helper lambda to draw text concisely
    auto drawText = [&](const std::string& str, float x, float y, unsigned int size, sf::Color color, bool bold = false) {
        if (!font_loaded) return;
        sf::Text text(font, str, size);
        text.setPosition({x, y});
        text.setFillColor(color);
        if (bold) text.setStyle(sf::Text::Bold);
        window.draw(text);
    };

    while (window.isOpen()) {
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }

            // Handle window resizing dynamically
            if (const auto* resized = event->getIf<sf::Event::Resized>()) {
                window.setView(sf::View(sf::FloatRect({0.f, 0.f}, sf::Vector2f(resized->size))));
            }

            // Keyboard controls for triggering algorithms
            if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
                if (keyPressed->code == sf::Keyboard::Key::Num1 || keyPressed->code == sf::Keyboard::Key::D) {
                    launchSearch(DIJKSTRA);
                } else if (keyPressed->code == sf::Keyboard::Key::Num2 || keyPressed->code == sf::Keyboard::Key::A) {
                    launchSearch(ASTAR);
                } else if (keyPressed->code == sf::Keyboard::Key::R || keyPressed->code == sf::Keyboard::Key::Space) {
                    system.stop_requested = true;
                    if (search_thread.joinable()) {
                        search_thread.join();
                    }
                    system.resetAllStats();
                }
            }
        }

        window.clear(sf::Color(16, 18, 24));

        float winWidth = static_cast<float>(window.getSize().x);
        float winHeight = static_cast<float>(window.getSize().y);

        // Layout: Dashboard occupies 400px on the right; Map occupies the remaining area on the left
        float panelWidth = 400.0f;
        float panelX = std::max(400.0f, winWidth - panelWidth);
        float mapAreaWidth = panelX;
        float mapAreaHeight = winHeight;

        // Strict 1:1 aspect ratio: cells are always true squares
        float cellSize = std::min(mapAreaWidth / system.width, mapAreaHeight / system.height);
        if (cellSize < 1.0f) cellSize = 1.0f;

        // Center the square map in the left area
        float offsetX = (mapAreaWidth - cellSize * system.width) / 2.0f;
        float offsetY = (mapAreaHeight - cellSize * system.height) / 2.0f;

        // ==========================================
        // 1. RENDER LEFT SIDE: 1:1 TERRAIN MAP
        // ==========================================
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

        // ==========================================
        // 2. RENDER RIGHT SIDE: ANALYTICS DASHBOARD (WHITE BACKGROUND)
        // ==========================================

        // Panel background card (Pure White)
        sf::RectangleShape panelBg({winWidth - panelX, winHeight});
        panelBg.setPosition({panelX, 0.f});
        panelBg.setFillColor(sf::Color(255, 255, 255));
        window.draw(panelBg);

        // Vertical divider separating map and dashboard
        sf::RectangleShape divider({2.0f, winHeight});
        divider.setPosition({panelX, 0.f});
        divider.setFillColor(sf::Color(218, 224, 233));
        window.draw(divider);

        float cardX = panelX + 20.0f;
        float cardW = 360.0f;

        // Title Header
        drawText("Project 2", cardX, 22.f, 22, sf::Color(25, 30, 42), true);
        drawText("Dijkstra vs A* Search Comparison", cardX, 50.f, 13, sf::Color(100, 110, 125));

        // --- Controls Card ---
        sf::RectangleShape controlsBox({cardW, 76.f});
        controlsBox.setPosition({cardX, 74.f});
        controlsBox.setFillColor(sf::Color(246, 248, 251));
        controlsBox.setOutlineColor(sf::Color(218, 224, 233));
        controlsBox.setOutlineThickness(1.f);
        window.draw(controlsBox);

        drawText("[1] Run Dijkstra's Algorithm", cardX + 15.f, 82.f, 13, sf::Color(15, 95, 175), true);
        drawText("[2] Run A* Search Algorithm", cardX + 15.f, 104.f, 13, sf::Color(20, 135, 55), true);
        drawText("[R] Reset Map & Comparison", cardX + 15.f, 126.f, 12, sf::Color(90, 100, 115));

        // --- Side-by-Side Comparison Table Card ---
        sf::RectangleShape tableCard({cardW, 260.f});
        tableCard.setPosition({cardX, 162.f});
        tableCard.setFillColor(sf::Color(246, 248, 251));
        tableCard.setOutlineColor(sf::Color(218, 224, 233));
        tableCard.setOutlineThickness(1.f);
        window.draw(tableCard);

        // Table Header
        drawText("METRIC", cardX + 12.f, 172.f, 12, sf::Color(100, 110, 125), true);
        drawText("DIJKSTRA", cardX + 135.f, 172.f, 13, sf::Color(15, 95, 175), true);
        drawText("A* SEARCH", cardX + 250.f, 172.f, 13, sf::Color(20, 135, 55), true);

        // Header divider
        sf::RectangleShape tableDivider({cardW - 20.f, 1.0f});
        tableDivider.setPosition({cardX + 10.f, 196.f});
        tableDivider.setFillColor(sf::Color(218, 224, 233));
        window.draw(tableDivider);

        // Table Row Renderer Helper
        auto drawTableRow = [&](const std::string& label, const std::string& dVal, const std::string& aVal, 
                                bool dActive, bool aActive, float yPos) {
            drawText(label, cardX + 12.f, yPos, 12, sf::Color(65, 75, 90));
            drawText(dVal, cardX + 135.f, yPos, 13, dActive ? sf::Color(20, 25, 35) : sf::Color(150, 160, 175), dActive);
            drawText(aVal, cardX + 250.f, yPos, 13, aActive ? sf::Color(20, 25, 35) : sf::Color(150, 160, 175), aActive);
        };

        const auto& d_stats = system.dijkstra_stats;
        const auto& a_stats = system.astar_stats;

        // Row 1: Status
        std::string dStatus = d_stats.is_running ? "Exploring..." : (d_stats.has_run ? "Done" : "---");
        std::string aStatus = a_stats.is_running ? "Exploring..." : (a_stats.has_run ? "Done" : "---");
        drawTableRow("Status", dStatus, aStatus, d_stats.has_run || d_stats.is_running, a_stats.has_run || a_stats.is_running, 206.f);

        // Row 2: Runtime
        std::string dTime = d_stats.has_run ? (std::to_string(static_cast<int>(d_stats.search_time_ms.load())) + " ms") : "---";
        std::string aTime = a_stats.has_run ? (std::to_string(static_cast<int>(a_stats.search_time_ms.load())) + " ms") : "---";
        drawTableRow("Runtime", dTime, aTime, d_stats.has_run, a_stats.has_run, 238.f);

        // Row 3: Nodes Explored
        std::string dNodes = d_stats.has_run ? std::to_string(d_stats.nodes_explored.load()) : "---";
        std::string aNodes = a_stats.has_run ? std::to_string(a_stats.nodes_explored.load()) : "---";
        drawTableRow("Nodes Visited", dNodes, aNodes, d_stats.has_run, a_stats.has_run, 270.f);

        // Row 4: Path Cost
        std::string dCost = d_stats.has_run ? (std::to_string(static_cast<int>(d_stats.total_cost.load())) + " m") : "---";
        std::string aCost = a_stats.has_run ? (std::to_string(static_cast<int>(a_stats.total_cost.load())) + " m") : "---";
        drawTableRow("Path Cost", dCost, aCost, d_stats.has_run, a_stats.has_run, 302.f);

        // Row 5: Path Length
        std::string dLen = d_stats.has_run ? (std::to_string(d_stats.path_length.load()) + " nodes") : "---";
        std::string aLen = a_stats.has_run ? (std::to_string(a_stats.path_length.load()) + " nodes") : "---";
        drawTableRow("Path Steps", dLen, aLen, d_stats.has_run, a_stats.has_run, 334.f);

        // Active State Indicator
        std::string activeInfo = system.is_searching ? 
            ("Active: " + std::string(system.current_algo == ASTAR ? "A* Search" : "Dijkstra") + " running...") :
            (d_stats.has_run || a_stats.has_run ? "Ready for next comparison" : "Press [1] or [2] to begin search");
        sf::Color activeColor = system.is_searching ? sf::Color(200, 110, 10) : sf::Color(80, 90, 105);
        drawText(activeInfo, cardX + 12.f, 395.f, 12, activeColor, true);

        // --- Dynamic Comparison Verdict Card ---
        float verdictH = (d_stats.has_run && a_stats.has_run) ? 122.f : 115.f;
        sf::RectangleShape verdictCard({cardW, verdictH});
        verdictCard.setPosition({cardX, 432.f});

        if (d_stats.has_run && a_stats.has_run) {
            int d_n = d_stats.nodes_explored.load();
            int a_n = a_stats.nodes_explored.load();
            double d_time = d_stats.search_time_ms.load();
            double a_time = a_stats.search_time_ms.load();
            double d_cost = d_stats.total_cost.load();
            double a_cost = a_stats.total_cost.load();
            int d_len = d_stats.path_length.load();
            int a_len = a_stats.path_length.load();

            bool cost_match = (d_len > 0 && a_len > 0 && std::abs(d_cost - a_cost) < 0.05);

            if (cost_match && a_n <= d_n) {
                verdictCard.setFillColor(sf::Color(238, 248, 240));
                verdictCard.setOutlineColor(sf::Color(55, 165, 85));
            } else {
                verdictCard.setFillColor(sf::Color(255, 250, 235));
                verdictCard.setOutlineColor(sf::Color(220, 160, 50));
            }
            verdictCard.setOutlineThickness(1.5f);
            window.draw(verdictCard);

            drawText("DYNAMIC VERDICT & BENCHMARK", cardX + 12.f, 439.f, 13, sf::Color(25, 125, 55), true);

            // 1. Exploration Efficiency (pruning)
            if (a_n < d_n) {
                int saved = d_n - a_n;
                double pct = (d_n > 0) ? (100.0 * saved / d_n) : 0.0;
                std::stringstream ss;
                ss << std::fixed << std::setprecision(1) << pct;
                drawText("* A* pruned " + std::to_string(saved) + " nodes (" + ss.str() + "% reduction)", 
                         cardX + 12.f, 460.f, 11, sf::Color(20, 135, 55), true);
            } else if (d_n < a_n) {
                int diff = a_n - d_n;
                drawText("* Dijkstra explored " + std::to_string(diff) + " fewer nodes", 
                         cardX + 12.f, 460.f, 11, sf::Color(15, 95, 175), true);
            } else {
                drawText("* Both algorithms explored identical " + std::to_string(a_n) + " nodes", 
                         cardX + 12.f, 460.f, 11, sf::Color(40, 50, 65));
            }

            // 2. Runtime Benchmark Comparison
            if (a_time > 0 && d_time > 0) {
                if (a_time < d_time) {
                    double speedup = d_time / a_time;
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(1) << speedup;
                    drawText("* A* ran " + ss.str() + "x faster (" + std::to_string(static_cast<int>(d_time - a_time)) + " ms saved)", 
                             cardX + 12.f, 480.f, 11, sf::Color(30, 40, 50));
                } else if (d_time < a_time) {
                    double speedup = a_time / d_time;
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(1) << speedup;
                    drawText("* Dijkstra ran " + ss.str() + "x faster (" + std::to_string(static_cast<int>(a_time - d_time)) + " ms saved)", 
                             cardX + 12.f, 480.f, 11, sf::Color(30, 40, 50));
                } else {
                    drawText("* Equal compute runtime (" + std::to_string(static_cast<int>(a_time)) + " ms)", 
                             cardX + 12.f, 480.f, 11, sf::Color(30, 40, 50));
                }
            } else {
                drawText("* Measured in sub-millisecond execution", cardX + 12.f, 480.f, 11, sf::Color(30, 40, 50));
            }

            // 3. Path Cost & Optimality Verification
            if (d_len > 0 && a_len > 0) {
                if (cost_match) {
                    drawText("* Both found identical lowest cost: " + std::to_string(static_cast<int>(a_cost)) + " m", 
                             cardX + 12.f, 500.f, 11, sf::Color(45, 75, 55));
                } else {
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(1) << std::abs(d_cost - a_cost);
                    drawText("* Cost difference: " + ss.str() + " m (Dijkstra: " + std::to_string(static_cast<int>(d_cost)) + " m / A*: " + std::to_string(static_cast<int>(a_cost)) + " m)", 
                             cardX + 12.f, 500.f, 11, sf::Color(180, 70, 20));
                }
            } else {
                drawText("* One or both algorithms found no valid path", cardX + 12.f, 500.f, 11, sf::Color(180, 50, 50));
            }

            // 4. Algorithmic Insight
            if (a_n < d_n) {
                drawText("* Distance heuristic guided A* straight to the goal", cardX + 12.f, 520.f, 11, sf::Color(80, 95, 110));
            } else {
                drawText("* Both algorithms explored similar search areas", cardX + 12.f, 520.f, 11, sf::Color(80, 95, 110));
            }
        } else if (d_stats.has_run && !a_stats.has_run) {
            verdictCard.setFillColor(sf::Color(246, 248, 251));
            verdictCard.setOutlineColor(sf::Color(218, 224, 233));
            verdictCard.setOutlineThickness(1.f);
            window.draw(verdictCard);

            drawText("COMPARISON STATUS", cardX + 12.f, 442.f, 13, sf::Color(15, 95, 175), true);
            drawText("Dijkstra explored " + std::to_string(d_stats.nodes_explored.load()) + " nodes (" + 
                     std::to_string(static_cast<int>(d_stats.search_time_ms.load())) + " ms).", 
                     cardX + 12.f, 468.f, 11, sf::Color(50, 60, 75));
            drawText("Now press [2] to run A* and unlock comparative analysis!", 
                     cardX + 12.f, 492.f, 11, sf::Color(20, 135, 55), true);
        } else if (!d_stats.has_run && a_stats.has_run) {
            verdictCard.setFillColor(sf::Color(246, 248, 251));
            verdictCard.setOutlineColor(sf::Color(218, 224, 233));
            verdictCard.setOutlineThickness(1.f);
            window.draw(verdictCard);

            drawText("COMPARISON STATUS", cardX + 12.f, 442.f, 13, sf::Color(20, 135, 55), true);
            drawText("A* explored " + std::to_string(a_stats.nodes_explored.load()) + " nodes (" + 
                     std::to_string(static_cast<int>(a_stats.search_time_ms.load())) + " ms).", 
                     cardX + 12.f, 468.f, 11, sf::Color(50, 60, 75));
            drawText("Now press [1] to run Dijkstra and unlock comparative analysis!", 
                     cardX + 12.f, 492.f, 11, sf::Color(15, 95, 175), true);
        } else {
            verdictCard.setFillColor(sf::Color(246, 248, 251));
            verdictCard.setOutlineColor(sf::Color(218, 224, 233));
            verdictCard.setOutlineThickness(1.f);
            window.draw(verdictCard);

            drawText("COMPARISON VERDICT", cardX + 12.f, 442.f, 13, sf::Color(100, 110, 125), true);
            drawText("Run BOTH Dijkstra [1] and A* [2] to", cardX + 12.f, 470.f, 12, sf::Color(115, 125, 140));
            drawText("unlock live speedup and pruning metrics.", cardX + 12.f, 492.f, 12, sf::Color(115, 125, 140));
        }

        // --- Map Terrain Legend Card ---
        sf::RectangleShape legendCard({cardW, 200.f});
        legendCard.setPosition({cardX, 565.f});
        legendCard.setFillColor(sf::Color(246, 248, 251));
        legendCard.setOutlineColor(sf::Color(218, 224, 233));
        legendCard.setOutlineThickness(1.f);
        window.draw(legendCard);

        drawText("MAP TERRAIN LEGEND", cardX + 12.f, 575.f, 13, sf::Color(65, 75, 90), true);

        auto drawLegendItem = [&](sf::Color color, const std::string& name, const std::string& cost, float yPos) {
            sf::RectangleShape swatch({14.f, 14.f});
            swatch.setPosition({cardX + 15.f, yPos + 2.f});
            swatch.setFillColor(color);
            swatch.setOutlineColor(sf::Color(180, 185, 195));
            swatch.setOutlineThickness(1.f);
            window.draw(swatch);

            drawText(name, cardX + 38.f, yPos, 12, sf::Color(30, 40, 55));
            drawText(cost, cardX + 270.f, yPos, 12, sf::Color(110, 120, 135));
        };

        drawLegendItem(sf::Color(0, 200, 0), "Trails (Footpaths)", "Cost: 1.0", 605.f);
        drawLegendItem(sf::Color(120, 180, 80), "Grass (Lawn)", "Cost: 2.5", 635.f);
        drawLegendItem(sf::Color(100, 100, 100), "Roads (Paved)", "Cost: 8.0", 665.f);
        drawLegendItem(sf::Color(180, 50, 50), "Buildings", "Cost: 60.0", 695.f);
        drawLegendItem(sf::Color(30, 30, 100), "Lake Alice (Water)", "Cost: 99.0", 725.f);

        window.display();
    }

    system.stop_requested = true;
    if (search_thread.joinable()) {
        search_thread.join();
    }

#ifdef _WIN32
    timeEndPeriod(1);
#endif

    // The console should not print anything until the application has closed.
    std::cout << system.console_log;

    return 0;
}
