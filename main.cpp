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

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

// Represents a single node in your 100,000+ node terrain grid
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

class TrailQuestSystem {
public:
    int width = 0;
    int height = 0;
    std::vector<Node> nodes;
    std::vector<std::atomic<NodeState>> node_states;

    TrailQuestSystem(const std::string& grid_filename) {
        loadGridFromFile(grid_filename);
    }

    // Load the 100,000+ node terrain grid exported by Python
    void loadGridFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open " << filename << ". Run your Python grid generator first!\n";
            return;
        }

        file >> width >> height;
        nodes.resize(width * height);
        node_states.assign(width * height, UNVISITED);

        int id;
        double x, y, weight;
        while (file >> id >> x >> y >> weight) {
            if (id >= 0 && id < nodes.size()) {
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

        std::cout << "Successfully loaded and wired " << nodes.size() << " nodes from " << filename << ".\n";
    }

    // Cost-based search simulation (Dijkstra using terrain weights)
    void runSearchSimulation(int start_idx, int end_idx) {
        if (nodes.empty()) return;

        using Element = std::pair<double, int>;
        std::priority_queue<Element, std::vector<Element>, std::greater<Element>> pq;

        std::vector<double> g_cost(nodes.size(), std::numeric_limits<double>::infinity());
        std::vector<bool> visited(nodes.size(), false);
        std::vector<int> came_from(nodes.size(), -1);

        g_cost[start_idx] = 0.0;
        pq.push({0.0, start_idx});
        node_states[start_idx] = FRONTIER;

        while (!pq.empty()) {
            auto [current_cost, current] = pq.top();
            pq.pop();

            if (visited[current]) continue;
            visited[current] = true;

            if (current == end_idx) {
                int curr = end_idx;
                while (curr != -1) {
                    node_states[curr] = PATH;
                    curr = came_from[curr];
                }
                std::cout << "Path found and highlighted on UI display!\n";
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
                    pq.push({tentative_gCost, neighbor});
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
};

int main() {
    TrailQuestSystem system("grid.txt");

    if (system.nodes.empty()) {
        std::cerr << "Grid failed to load. Ensure grid.txt is in the same directory.\n";
        return -1;
    }

    sf::RenderWindow window(sf::VideoMode({1000, 400}), "TrailQuest - OSM Terrain & Search Frontier Visualizer");
    window.setFramerateLimit(60);

    int start_node = 20 * system.width + 20;
    int end_node = (system.height - 20) * system.width + (system.width - 20);

#ifdef _WIN32
    timeBeginPeriod(1); // added this so the animation paces out better on Windows
#endif

    std::thread search_thread(&TrailQuestSystem::runSearchSimulation, &system, start_node, end_node);

    while (window.isOpen()) {
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }

        window.clear(sf::Color::Black);

        float cellWidth = static_cast<float>(window.getSize().x) / system.width;
        float cellHeight = static_cast<float>(window.getSize().y) / system.height;
        sf::RectangleShape rect(sf::Vector2f(cellWidth > 1 ? cellWidth : 1, cellHeight > 1 ? cellHeight : 1));

        for (int y = 0; y < system.height; ++y) {
            for (int x = 0; x < system.width; ++x) {
                int idx = y * system.width + x;
                rect.setPosition({x * cellWidth, y * cellHeight});

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

        window.display();
    }

    if (search_thread.joinable()) {
        search_thread.join();
    }

#ifdef _WIN32
    timeEndPeriod(1);
#endif

    return 0;
}