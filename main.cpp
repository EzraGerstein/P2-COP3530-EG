
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
#include <algorithm>
#include <mutex>

// Represents a single node in your terrain grid
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
    FRONTIER,
    VISITED,
    PATH
};

class TrailQuestSystem {
public:
    int width = 0;
    int height = 0;

    std::vector<Node> nodes;
    std::vector<NodeState> node_states;
    std::mutex state_mutex;

    TrailQuestSystem(const std::string& grid_filename) {
        loadGridFromFile(grid_filename);
    }

    void loadGridFromFile(const std::string& filename) {
        std::ifstream file(filename);

        if (!file.is_open()) {
            std::cerr << "Could not open " << filename << "\n";
            return;
        }

        file >> width >> height;

        if (!file || width <= 0 || height <= 0) {
            std::cerr << "Invalid grid dimensions.\n";
            width = height = 0;
            return;
        }

        nodes.resize(static_cast<size_t>(width) * height);
        node_states.assign(nodes.size(), UNVISITED);

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

        // Create grid connections
        for (int y_idx = 0; y_idx < height; ++y_idx) {
            for (int x_idx = 0; x_idx < width; ++x_idx) {
                int idx = y_idx * width + x_idx;

                if (x_idx > 0)
                    nodes[idx].neighbors.push_back(idx - 1);

                if (x_idx < width - 1)
                    nodes[idx].neighbors.push_back(idx + 1);

                if (y_idx > 0)
                    nodes[idx].neighbors.push_back(idx - width);

                if (y_idx < height - 1)
                    nodes[idx].neighbors.push_back(idx + width);
            }
        }

        std::cout << "Successfully loaded "
                  << nodes.size() << " nodes.\n";
    }

    // Original Dijkstra search
    void runSearchSimulation(int start_idx, int end_idx) {
        if (nodes.empty()) return;

        using Element = std::pair<double, int>;

        std::priority_queue<
            Element,
            std::vector<Element>,
            std::greater<Element>
        > pq;

        std::vector<double> g_cost(
            nodes.size(),
            std::numeric_limits<double>::infinity()
        );

        std::vector<bool> visited(nodes.size(), false);
        std::vector<int> came_from(nodes.size(), -1);

        g_cost[start_idx] = 0.0;
        pq.push({0.0, start_idx});

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            node_states[start_idx] = FRONTIER;
        }

        while (!pq.empty()) {
            auto [current_cost, current] = pq.top();
            pq.pop();

            if (visited[current]) continue;

            visited[current] = true;

            if (current == end_idx) {
                int curr = end_idx;

                std::lock_guard<std::mutex> lock(state_mutex);

                while (curr != -1) {
                    node_states[curr] = PATH;
                    curr = came_from[curr];
                }

                std::cout << "Path found!\n";
                return;
            }

            {
                std::lock_guard<std::mutex> lock(state_mutex);
                node_states[current] = VISITED;
            }

            for (int neighbor : nodes[current].neighbors) {
                if (visited[neighbor]) continue;

                if (nodes[neighbor].terrain_weight >= 99.0)
                    continue;

                double weight = nodes[neighbor].terrain_weight;
                double tentative_gCost = current_cost + weight;

                if (tentative_gCost < g_cost[neighbor]) {
                    g_cost[neighbor] = tentative_gCost;
                    came_from[neighbor] = current;
                    pq.push({tentative_gCost, neighbor});

                    std::lock_guard<std::mutex> lock(state_mutex);
                    node_states[neighbor] = FRONTIER;
                }
            }

            std::this_thread::sleep_for(
                std::chrono::microseconds(30)
            );
        }

        std::cout << "No path found.\n";
    }
};

int main() {
    TrailQuestSystem system("grid.txt");

    if (system.nodes.empty()) {
        std::cerr << "Grid failed to load.\n";
        return -1;
    }

    // Window
    const int WINDOW_WIDTH = 1200;
    const int WINDOW_HEIGHT = 700;

    sf::RenderWindow window(
        sf::VideoMode({WINDOW_WIDTH, WINDOW_HEIGHT}),
        "TrailQuest - Hiking Navigation"
    );

    window.setFramerateLimit(60);

    // Start and end points
    int start_node =
        std::min(20, system.height - 1) * system.width +
        std::min(20, system.width - 1);

    int end_node =
        (system.height - 1 - std::min(20, system.height - 1))
        * system.width +
        (system.width - 1 - std::min(20, system.width - 1));

    // Terrain colors
    auto getTerrainColor = [](double weight) {
        if (weight >= 99.0)
            return sf::Color(65, 130, 180);

        if (weight >= 50.0)
            return sf::Color(155, 105, 90);

        if (weight >= 5.0)
            return sf::Color(195, 185, 160);

        if (weight >= 2.0)
            return sf::Color(155, 180, 120);

        return sf::Color(65, 115, 75);
    };

    // Map settings
    float cellWidth =
        static_cast<float>(WINDOW_WIDTH) / system.width;

    float cellHeight =
        static_cast<float>(WINDOW_HEIGHT - 50) / system.height;

    sf::RectangleShape rect({
        cellWidth + 0.2f,
        cellHeight + 0.2f
    });

    // Start marker
    sf::CircleShape startMarker(5.f);
    startMarker.setOrigin({5.f, 5.f});
    startMarker.setFillColor(sf::Color(60, 220, 120));
    startMarker.setOutlineColor(sf::Color::White);
    startMarker.setOutlineThickness(2.f);

    startMarker.setPosition({
        (start_node % system.width + 0.5f) * cellWidth,
        50.f + (start_node / system.width + 0.5f) * cellHeight
    });

    // Destination marker
    sf::CircleShape endMarker(5.f);
    endMarker.setOrigin({5.f, 5.f});
    endMarker.setFillColor(sf::Color(235, 75, 75));
    endMarker.setOutlineColor(sf::Color::White);
    endMarker.setOutlineThickness(2.f);

    endMarker.setPosition({
        (end_node % system.width + 0.5f) * cellWidth,
        50.f + (end_node / system.width + 0.5f) * cellHeight
    });

    // Font
    sf::Font font;
    bool fontLoaded = font.openFromFile(
        "C:/Windows/Fonts/arial.ttf"
    );


    // BUTTONS


    sf::RectangleShape dijkstraButton({140.f, 40.f});
    dijkstraButton.setPosition({20.f, 5.f});
    dijkstraButton.setFillColor(sf::Color(65, 125, 75));

    sf::RectangleShape astarButton({140.f, 40.f});
    astarButton.setPosition({175.f, 5.f});
    astarButton.setFillColor(sf::Color(65, 85, 110));

    sf::RectangleShape startButton({140.f, 40.f});
    startButton.setPosition({330.f, 5.f});
    startButton.setFillColor(sf::Color(65, 155, 85));

    sf::RectangleShape resetButton({140.f, 40.f});
    resetButton.setPosition({485.f, 5.f});
    resetButton.setFillColor(sf::Color(100, 100, 100));

    bool useDijkstra = true;
    bool searchStarted = false;

    std::thread searchThread;


    // MAIN DISPLAY LOOP


    while (window.isOpen()) {

        while (const std::optional event = window.pollEvent()) {

            if (event->is<sf::Event::Closed>()) {
                window.close();
            }

            // Button clicks
            if (const auto* mouse =
                event->getIf<sf::Event::MouseButtonPressed>()) {

                if (mouse->button == sf::Mouse::Button::Left) {

                    sf::Vector2f pos(
                        static_cast<float>(mouse->position.x),
                        static_cast<float>(mouse->position.y)
                    );

                    if (!searchStarted) {

                        if (dijkstraButton.getGlobalBounds().contains(pos)) {
                            useDijkstra = true;
                            std::cout << "Dijkstra selected\n";
                        }

                        if (astarButton.getGlobalBounds().contains(pos)) {
                            useDijkstra = false;
                            std::cout << "A* selected\n";
                        }

                        if (startButton.getGlobalBounds().contains(pos)) {

                            if (useDijkstra) {
                                searchStarted = true;

                                searchThread = std::thread(
                                    &TrailQuestSystem::runSearchSimulation,
                                    &system,
                                    start_node,
                                    end_node
                                );
                            }
                            else {
                                std::cout << "Waiting for partner's A* function.\n";
                            }
                        }

                        if (resetButton.getGlobalBounds().contains(pos)) {
                            std::lock_guard<std::mutex> lock(
                                system.state_mutex
                            );

                            std::fill(
                                system.node_states.begin(),
                                system.node_states.end(),
                                UNVISITED
                            );

                            std::cout << "Map reset.\n";
                        }
                    }
                }
            }
        }

        window.clear(sf::Color(35, 48, 40));

        // Draw map
        {
            std::lock_guard<std::mutex> lock(system.state_mutex);

            for (int y = 0; y < system.height; ++y) {
                for (int x = 0; x < system.width; ++x) {

                    int idx = y * system.width + x;

                    rect.setPosition({
                        x * cellWidth,
                        50.f + y * cellHeight
                    });

                    switch (system.node_states[idx]) {

                        case FRONTIER:
                            rect.setFillColor(sf::Color(245, 195, 80));
                            break;

                        case VISITED:
                            rect.setFillColor(sf::Color(100, 145, 190));
                            break;

                        case PATH:
                            rect.setFillColor(sf::Color(255, 125, 45));
                            break;

                        case UNVISITED:
                            rect.setFillColor(
                                getTerrainColor(
                                    system.nodes[idx].terrain_weight
                                )
                            );
                            break;
                    }

                    window.draw(rect);
                }
            }
        }

        // Draw markers
        window.draw(startMarker);
        window.draw(endMarker);

        // Draw buttons
        window.draw(dijkstraButton);
        window.draw(astarButton);
        window.draw(startButton);
        window.draw(resetButton);

        // Button labels
        if (fontLoaded) {

            sf::Text dijkstraText(font, "DIJKSTRA", 16);
            dijkstraText.setPosition({35.f, 15.f});
            window.draw(dijkstraText);

            sf::Text astarText(font, "A*", 16);
            astarText.setPosition({230.f, 15.f});
            window.draw(astarText);

            sf::Text startText(font, "START", 16);
            startText.setPosition({375.f, 15.f});
            window.draw(startText);

            sf::Text resetText(font, "RESET", 16);
            resetText.setPosition({525.f, 15.f});
            window.draw(resetText);
        }

        window.display();
    }

    if (searchThread.joinable()) {
        searchThread.join();
    }

    return 0;
}
