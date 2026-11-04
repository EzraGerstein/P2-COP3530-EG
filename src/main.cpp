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

struct Node {
    int node_id;
    double x_coord;
    double y_coord;
    double terrain_weight;
    std::vector<int> neighbors;
};

enum NodeState {
    UNVISITED,
    FRONTIER,
    VISITED,
    PATH
};

enum Algorithm {
    DIJKSTRA,
    ASTAR
};

struct AlgorithmStats {
    std::atomic<bool> has_run{false};
    std::atomic<bool> is_running{false};
    std::atomic<double> search_time_ms{0.0};
    std::atomic<int> nodes_explored{0};
    std::atomic<double> total_cost{0.0};
    std::atomic<int> path_length{0};
};

struct VisualHeapElement {
    int node_id;
    double priority;
};

// min-heap with pushOrDecreaseKey for the visualizer
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
        if (width <= 0 || height <= 0) {
            generateDefaultSquareGrid(320, 320);
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

    // generates a fallback campus grid if grid.txt is not present
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

                // base grass weight
                double weight = 2.5;

                // lake alice
                double dx = (c - 50.0) / 20.0;
                double dy = (r - 50.0) / 14.0;
                double lake_dist = dx * dx + dy * dy;
                if (lake_dist <= 1.0) {
                    weight = 99.0;
                }

                // small pond
                double px = (c - 76.0) / 8.0;
                double py = (r - 28.0) / 6.0;
                if (px * px + py * py <= 1.0) {
                    weight = 99.0;
                }

                // campus buildings
                if ((r >= 10 && r <= 20 && c >= 28 && c <= 42) ||
                    (r >= 12 && r <= 24 && c >= 62 && c <= 76) ||
                    (r >= 74 && r <= 86 && c >= 18 && c <= 32) ||
                    (r >= 70 && r <= 82 && c >= 64 && c <= 78)) {
                    weight = 60.0;
                }

                // perimeter roads
                if ((c == 15 || c == 85) && r >= 5 && r <= 95) weight = 8.0;
                if ((r == 15 || r == 85) && c >= 5 && c <= 95) weight = 8.0;

                // trails around lake and connecting paths
                if (lake_dist > 1.0 && lake_dist <= 1.35) {
                    weight = 1.0;
                }
                if (std::abs(r - c) <= 1 && (r <= 36 || r >= 64)) {
                    weight = 1.0;
                }
                if (c == 28 && r >= 36 && r <= 64) weight = 1.0;
                if (c == 72 && r >= 36 && r <= 64) weight = 1.0;

                nodes[id].terrain_weight = weight;
            }
        }

        // connect 4-way neighbors
        for (int y_idx = 0; y_idx < height; ++y_idx) {
            for (int x_idx = 0; x_idx < width; ++x_idx) {
                int idx = y_idx * width + x_idx;
                if (x_idx > 0) nodes[idx].neighbors.push_back(idx - 1);           // left
                if (x_idx < width - 1) nodes[idx].neighbors.push_back(idx + 1);       // right
                if (y_idx > 0) nodes[idx].neighbors.push_back(idx - width);       // up
                if (y_idx < height - 1) nodes[idx].neighbors.push_back(idx + width);  // down
            }
        }

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

    // load grid data from file
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
        if (width <= 0 || height <= 0 || width != height) {
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

        // connect 4-way neighbors
        for (int y_idx = 0; y_idx < height; ++y_idx) {
            for (int x_idx = 0; x_idx < width; ++x_idx) {
                int idx = y_idx * width + x_idx;
                if (x_idx > 0) nodes[idx].neighbors.push_back(idx - 1);           // left
                if (x_idx < width - 1) nodes[idx].neighbors.push_back(idx + 1);       // Right
                if (y_idx > 0) nodes[idx].neighbors.push_back(idx - width);       // Up
                if (y_idx < height - 1) nodes[idx].neighbors.push_back(idx + width);  // Down
            }
        }

        console_log += "Successfully loaded and wired " + std::to_string(nodes.size()) + " nodes from " + filename + ".\n";
    }

    void runSearchSimulation(Algorithm algo, int start_idx, int end_idx) {
        if (nodes.empty()) return;

        stop_requested = false;
        is_searching = true;
        current_algo = algo;
        nodes_explored = 0;
        final_path_distance = 0.0;

        if (algo == DIJKSTRA) dijkstra_stats.is_running = true;
        if (algo == ASTAR) astar_stats.is_running = true;

        // benchmark raw search time first without GUI delay
        double pure_search_time_ms = 0.0;
        {
            auto bench_start = std::chrono::high_resolution_clock::now();
            VisualMinHeap bench_pq(nodes.size());
            std::vector<double> bench_g(nodes.size(), std::numeric_limits<double>::infinity());
            std::vector<bool> bench_vis(nodes.size(), false);

            bench_g[start_idx] = 0.0;
            double p0 = (algo == ASTAR) ? calculateHeuristic(nodes[start_idx], nodes[end_idx]) : 0.0;
            bench_pq.pushOrDecreaseKey(start_idx, p0);

            while (!bench_pq.empty()) {
                VisualHeapElement elem = bench_pq.extractMin();
                int u = elem.node_id;
                if (bench_vis[u]) continue;
                bench_vis[u] = true;
                if (u == end_idx) break;

                for (int v : nodes[u].neighbors) {
                    if (bench_vis[v] || nodes[v].terrain_weight >= 99.0) continue;
                    double cost = bench_g[u] + nodes[v].terrain_weight;
                    if (cost < bench_g[v]) {
                        bench_g[v] = cost;
                        double p = cost + ((algo == ASTAR) ? calculateHeuristic(nodes[v], nodes[end_idx]) : 0.0);
                        bench_pq.pushOrDecreaseKey(v, p);
                    }
                }
            }
            auto bench_end = std::chrono::high_resolution_clock::now();
            pure_search_time_ms = std::chrono::duration<double, std::milli>(bench_end - bench_start).count();
            if (pure_search_time_ms < 0.05) pure_search_time_ms = 0.05;
        }

        // visual pass for the animation
        VisualMinHeap pq(nodes.size());

        std::vector<double> g_cost(nodes.size(), std::numeric_limits<double>::infinity());
        std::vector<bool> visited(nodes.size(), false);
        std::vector<int> came_from(nodes.size(), -1);

        g_cost[start_idx] = 0.0;
        double start_priority = (algo == ASTAR) ? calculateHeuristic(nodes[start_idx], nodes[end_idx]) : 0.0;
        pq.pushOrDecreaseKey(start_idx, start_priority);
        node_states[start_idx] = FRONTIER;

        const int BATCH_SIZE = (algo == ASTAR) ? 8 : 25;
        const int BATCH_PAUSE_MS = (algo == ASTAR) ? 12 : 10;
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
                std::vector<int> path_nodes;
                int curr = end_idx;
                while (curr != -1) {
                    path_nodes.push_back(curr);
                    curr = came_from[curr];
                }
                final_path_distance = g_cost[end_idx];

                if (algo == DIJKSTRA) {
                    dijkstra_stats.search_time_ms = pure_search_time_ms;
                    dijkstra_stats.nodes_explored = nodes_explored.load();
                    dijkstra_stats.total_cost = final_path_distance.load();
                    dijkstra_stats.path_length = static_cast<int>(path_nodes.size());
                    dijkstra_stats.has_run = true;
                    dijkstra_stats.is_running = false;
                } else {
                    astar_stats.search_time_ms = pure_search_time_ms;
                    astar_stats.nodes_explored = nodes_explored.load();
                    astar_stats.total_cost = final_path_distance.load();
                    astar_stats.path_length = static_cast<int>(path_nodes.size());
                    astar_stats.has_run = true;
                    astar_stats.is_running = false;
                }

                std::string algo_str = (algo == ASTAR) ? "A* Search" : "Dijkstra";
                console_log += algo_str + " finished: Path found! Visited " + std::to_string(nodes_explored.load()) +
                               " nodes. Total cost: " + std::to_string(final_path_distance.load()) + "\n";

                // animate the path
                std::reverse(path_nodes.begin(), path_nodes.end());
                for (int p_node : path_nodes) {
                    if (stop_requested) break;
                    node_states[p_node] = PATH;
                    std::this_thread::sleep_for(std::chrono::milliseconds(8));
                }
                break;
            }

            node_states[current] = VISITED;

            for (int neighbor : nodes[current].neighbors) {
                if (visited[neighbor]) continue;

                // impassable water
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

    // map on the left, stats on the right
    sf::RenderWindow window(sf::VideoMode({1200, 800}), "Project 2");
    window.setFramerateLimit(60);

    // try to find a font file
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

    enum SelectionMode { MODE_NONE, MODE_SET_START, MODE_SET_GOAL };
    SelectionMode select_mode = MODE_NONE;

    struct ButtonDef {
        float x, y, w, h;
        bool contains(const sf::Vector2f& pt) const {
            return pt.x >= x && pt.x <= x + w && pt.y >= y && pt.y <= y + h;
        }
    };

#ifdef _WIN32
    timeBeginPeriod(1); // added this so the animation paces out better on Windows
#endif

    std::thread search_thread;

    auto cancelSearch = [&]() {
        system.stop_requested = true;
        if (search_thread.joinable()) {
            search_thread.join();
        }
        system.resetStates();
    };

    auto launchSearch = [&](Algorithm algo) {
        cancelSearch();
        search_thread = std::thread(&TrailQuestSystem::runSearchSimulation, &system, algo, start_node, end_node);
    };

    auto drawText = [&](const std::string& str, float x, float y, unsigned int size, sf::Color color, bool bold = false) {
        if (!font_loaded) return;
        sf::Text text(font, str, size);
        text.setPosition({x, y});
        text.setFillColor(color);
        if (bold) text.setStyle(sf::Text::Bold);
        window.draw(text);
    };

    auto drawButton = [&](const ButtonDef& btn, const std::string& label, sf::Color bgColor, 
                          sf::Color textColor, sf::Color borderColor, float borderThick = 1.0f, bool bold = false) {
        sf::RectangleShape box({btn.w, btn.h});
        box.setPosition({btn.x, btn.y});
        box.setFillColor(bgColor);
        box.setOutlineColor(borderColor);
        box.setOutlineThickness(borderThick);
        window.draw(box);

        if (font_loaded) {
            sf::Text txt(font, label, 12);
            if (bold) txt.setStyle(sf::Text::Bold);
            txt.setFillColor(textColor);
            sf::FloatRect bounds = txt.getLocalBounds();
            float tx = btn.x + (btn.w - bounds.size.x) / 2.0f - bounds.position.x;
            float ty = btn.y + (btn.h - bounds.size.y) / 2.0f - bounds.position.y;
            txt.setPosition({tx, ty});
            window.draw(txt);
        }
    };

    while (window.isOpen()) {
        float winWidth = static_cast<float>(window.getSize().x);
        float winHeight = static_cast<float>(window.getSize().y);

        float panelWidth = 400.0f;
        float panelX = std::max(400.0f, winWidth - panelWidth);
        float mapAreaWidth = panelX;
        float mapAreaHeight = winHeight;

        // square cells for 1:1 map
        float cellSize = std::min(mapAreaWidth / system.width, mapAreaHeight / system.height);
        if (cellSize < 1.0f) cellSize = 1.0f;

        float offsetX = (mapAreaWidth - cellSize * system.width) / 2.0f;
        float offsetY = (mapAreaHeight - cellSize * system.height) / 2.0f;

        float cardX = panelX + 20.0f;
        float cardW = 360.0f;

        ButtonDef btnDijkstra{cardX + 8.f, 68.f, 168.f, 32.f};
        ButtonDef btnAStar{cardX + 184.f, 68.f, 168.f, 32.f};
        ButtonDef btnReset{cardX + 8.f, 106.f, 344.f, 24.f};

        ButtonDef btnSetStart{cardX + 8.f, 194.f, 108.f, 28.f};
        ButtonDef btnSetGoal{cardX + 124.f, 194.f, 108.f, 28.f};
        ButtonDef btnPresetLake{cardX + 240.f, 194.f, 112.f, 28.f};

        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }

            if (const auto* resized = event->getIf<sf::Event::Resized>()) {
                window.setView(sf::View(sf::FloatRect({0.f, 0.f}, sf::Vector2f(resized->size))));
            }

            if (const auto* mousePressed = event->getIf<sf::Event::MouseButtonPressed>()) {
                sf::Vector2f clickPos = window.mapPixelToCoords(mousePressed->position);

                // click on map to choose start/end node
                if (clickPos.x >= offsetX && clickPos.x < offsetX + cellSize * system.width &&
                    clickPos.y >= offsetY && clickPos.y < offsetY + cellSize * system.height) {
                    int cellX = static_cast<int>((clickPos.x - offsetX) / cellSize);
                    int cellY = static_cast<int>((clickPos.y - offsetY) / cellSize);
                    cellX = std::clamp(cellX, 0, system.width - 1);
                    cellY = std::clamp(cellY, 0, system.height - 1);
                    int clicked_node = cellY * system.width + cellX;

                    bool isLeft = (mousePressed->button == sf::Mouse::Button::Left);
                    bool isRight = (mousePressed->button == sf::Mouse::Button::Right);

                    if (select_mode == MODE_SET_START || (isLeft && select_mode != MODE_SET_GOAL)) {
                        cancelSearch();
                        start_node = clicked_node;
                        system.resetAllStats();
                        select_mode = MODE_NONE;
                    } else if (select_mode == MODE_SET_GOAL || isRight) {
                        cancelSearch();
                        end_node = clicked_node;
                        system.resetAllStats();
                        select_mode = MODE_NONE;
                    }
                }
                // click dashboard buttons
                else if (mousePressed->button == sf::Mouse::Button::Left) {
                    if (btnDijkstra.contains(clickPos)) {
                        select_mode = MODE_NONE;
                        launchSearch(DIJKSTRA);
                    } else if (btnAStar.contains(clickPos)) {
                        select_mode = MODE_NONE;
                        launchSearch(ASTAR);
                    } else if (btnReset.contains(clickPos)) {
                        select_mode = MODE_NONE;
                        cancelSearch();
                        system.resetAllStats();
                    } else if (btnSetStart.contains(clickPos)) {
                        select_mode = (select_mode == MODE_SET_START) ? MODE_NONE : MODE_SET_START;
                    } else if (btnSetGoal.contains(clickPos)) {
                        select_mode = (select_mode == MODE_SET_GOAL) ? MODE_NONE : MODE_SET_GOAL;
                    } else if (btnPresetLake.contains(clickPos)) {
                        select_mode = MODE_NONE;
                        cancelSearch();
                        start_node = 32 * system.width + 18;  // west of Lake Alice
                        end_node = 68 * system.width + 82;    // east of Lake Alice
                        system.resetAllStats();
                    }
                }
            }
        }

        window.clear(sf::Color(16, 18, 24));

        // map border
        sf::RectangleShape mapBorder({cellSize * system.width + 4.f, cellSize * system.height + 4.f});
        mapBorder.setPosition({offsetX - 2.f, offsetY - 2.f});
        mapBorder.setFillColor(sf::Color(20, 24, 32));
        mapBorder.setOutlineColor(sf::Color(45, 52, 65));
        mapBorder.setOutlineThickness(1.5f);
        window.draw(mapBorder);

        // vertex array for fast grid drawing
        sf::VertexArray terrainVA(sf::PrimitiveType::Triangles, system.width * system.height * 6);
        sf::VertexArray searchOverlayVA(sf::PrimitiveType::Triangles);

        for (int y = 0; y < system.height; ++y) {
            for (int x = 0; x < system.width; ++x) {
                int idx = y * system.width + x;
                float px = offsetX + x * cellSize;
                float py = offsetY + y * cellSize;

                sf::Color baseColor;
                double w = system.nodes[idx].terrain_weight;
                if (w == 1.0) {
                    baseColor = sf::Color(38, 135, 65);       // trails
                } else if (w == 2.5) {
                    baseColor = sf::Color(122, 158, 102);     // grass
                } else if (w == 8.0) {
                    baseColor = sf::Color(65, 72, 85);        // roads
                } else if (w == 60.0) {
                    baseColor = sf::Color(148, 62, 50);       // buildings
                } else {
                    // shoreline effect for water
                    bool isShore = false;
                    for (int n : system.nodes[idx].neighbors) {
                        if (system.nodes[n].terrain_weight < 90.0) { isShore = true; break; }
                    }
                    baseColor = isShore ? sf::Color(38, 92, 135) : sf::Color(22, 55, 86);
                }

                // quad vertices
                int vIdx = idx * 6;
                terrainVA[vIdx + 0] = {{px, py}, baseColor};
                terrainVA[vIdx + 1] = {{px + cellSize, py}, baseColor};
                terrainVA[vIdx + 2] = {{px + cellSize, py + cellSize}, baseColor};
                terrainVA[vIdx + 3] = {{px, py}, baseColor};
                terrainVA[vIdx + 4] = {{px + cellSize, py + cellSize}, baseColor};
                terrainVA[vIdx + 5] = {{px, py + cellSize}, baseColor};

                // search and path overlay
                NodeState state = system.node_states[idx];
                if (state != UNVISITED) {
                    sf::Color overlayColor;
                    if (state == FRONTIER) {
                        overlayColor = sf::Color(255, 215, 0, 220);     // frontier
                    } else if (state == VISITED) {
                        overlayColor = sf::Color(80, 165, 245, 125);    // visited
                    } else if (state == PATH) {
                        overlayColor = sf::Color(255, 45, 85, 245);     // shortest path
                    }

                    searchOverlayVA.append({{px, py}, overlayColor});
                    searchOverlayVA.append({{px + cellSize, py}, overlayColor});
                    searchOverlayVA.append({{px + cellSize, py + cellSize}, overlayColor});
                    searchOverlayVA.append({{px, py}, overlayColor});
                    searchOverlayVA.append({{px + cellSize, py + cellSize}, overlayColor});
                    searchOverlayVA.append({{px, py + cellSize}, overlayColor});
                }
            }
        }

        window.draw(terrainVA);
        if (searchOverlayVA.getVertexCount() > 0) {
            window.draw(searchOverlayVA);
        }

        // compass
        float compassX = offsetX + cellSize * system.width - 32.f;
        float compassY = offsetY + 30.f;

        sf::ConvexShape northArrow(3);
        northArrow.setPoint(0, {compassX, compassY - 14.f});
        northArrow.setPoint(1, {compassX - 6.f, compassY});
        northArrow.setPoint(2, {compassX + 6.f, compassY});
        northArrow.setFillColor(sf::Color(220, 45, 45));
        window.draw(northArrow);

        sf::ConvexShape southArrow(3);
        southArrow.setPoint(0, {compassX - 6.f, compassY});
        southArrow.setPoint(1, {compassX, compassY + 12.f});
        southArrow.setPoint(2, {compassX + 6.f, compassY});
        southArrow.setFillColor(sf::Color(160, 170, 185));
        window.draw(southArrow);

        drawText("N", compassX - 4.f, compassY - 26.f, 10, sf::Color::White);

        // // scale bar
        // float scaleBarX = offsetX + 14.f;
        // float scaleBarY = offsetY + cellSize * system.height - 24.f;
        // float scaleBarW = cellSize * 20.f;
        //
        // sf::RectangleShape scaleBarBg({scaleBarW + 55.f, 18.f});
        // scaleBarBg.setPosition({scaleBarX - 4.f, scaleBarY - 2.f});
        // scaleBarBg.setFillColor(sf::Color(18, 22, 30, 190));
        // window.draw(scaleBarBg);

        // sf::RectangleShape scaleLine({scaleBarW, 2.f});
        // scaleLine.setPosition({scaleBarX, scaleBarY + 8.f});
        // scaleLine.setFillColor(sf::Color::White);
        // window.draw(scaleLine);
        //
        // sf::RectangleShape scaleTick1({2.f, 6.f});
        // scaleTick1.setPosition({scaleBarX, scaleBarY + 6.f});
        // scaleTick1.setFillColor(sf::Color::White);
        // window.draw(scaleTick1);

        // sf::RectangleShape scaleTick2({2.f, 6.f});
        // scaleTick2.setPosition({scaleBarX + scaleBarW, scaleBarY + 6.f});
        // scaleTick2.setFillColor(sf::Color::White);
        // window.draw(scaleTick2);
        //
        // drawText("200 m", scaleBarX + scaleBarW + 6.f, scaleBarY + 3.f, 9, sf::Color(220, 225, 235));

        // start marker
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

        // destination marker
        float endX = offsetX + (end_node % system.width + 0.5f) * cellSize;
        float endY = offsetY + (end_node / system.width + 0.5f) * cellSize;

        float ringRadius = std::max(6.0f, cellSize * 1.3f);
        sf::CircleShape goalRing(ringRadius);
        goalRing.setOrigin({ringRadius, ringRadius});
        goalRing.setPosition({endX, endY});
        goalRing.setFillColor(sf::Color::Transparent);
        goalRing.setOutlineColor(sf::Color(255, 50, 50, 220));
        goalRing.setOutlineThickness(std::max(2.0f, cellSize * 0.25f));
        window.draw(goalRing);

        float poleW = std::max(2.5f, cellSize * 0.25f);
        float poleH = std::max(18.0f, cellSize * 2.8f);
        sf::RectangleShape pole(sf::Vector2f(poleW, poleH));
        pole.setFillColor(sf::Color(240, 240, 240));
        pole.setOutlineColor(sf::Color(30, 30, 30));
        pole.setOutlineThickness(1.0f);
        pole.setPosition({endX - poleW / 2.0f, endY - poleH + ringRadius * 0.3f});
        window.draw(pole);

        float ballRadius = std::max(2.5f, poleW * 1.2f);
        sf::CircleShape finial(ballRadius);
        finial.setOrigin({ballRadius, ballRadius});
        finial.setPosition({pole.getPosition().x + poleW / 2.0f, pole.getPosition().y});
        finial.setFillColor(sf::Color(255, 215, 0));
        window.draw(finial);

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

        // dashboard on right side
        sf::RectangleShape panelBg({winWidth - panelX, winHeight});
        panelBg.setPosition({panelX, 0.f});
        panelBg.setFillColor(sf::Color(255, 255, 255));
        window.draw(panelBg);

        // divider line
        sf::RectangleShape divider({2.0f, winHeight});
        divider.setPosition({panelX, 0.f});
        divider.setFillColor(sf::Color(218, 224, 233));
        window.draw(divider);

        // title
        drawText("Project 2", cardX, 18.f, 22, sf::Color(25, 30, 42));
        drawText("Dijkstra vs A* Search Comparison", cardX, 44.f, 12, sf::Color(100, 110, 125));

        sf::Vector2i mousePixel = sf::Mouse::getPosition(window);
        sf::Vector2f mousePos = window.mapPixelToCoords(mousePixel);

        // run buttons box
        sf::RectangleShape runBox({cardW, 76.f});
        runBox.setPosition({cardX, 62.f});
        runBox.setFillColor(sf::Color(246, 248, 251));
        runBox.setOutlineColor(sf::Color(218, 224, 233));
        runBox.setOutlineThickness(1.f);
        window.draw(runBox);

        bool dHover = btnDijkstra.contains(mousePos);
        drawButton(btnDijkstra, "Run Dijkstra",
                   dHover ? sf::Color(21, 101, 192) : sf::Color(25, 118, 210), 
                   sf::Color::White, sf::Color(15, 80, 160), 1.0f, false);

        bool aHover = btnAStar.contains(mousePos);
        drawButton(btnAStar, "Run A* Search",
                   aHover ? sf::Color(27, 94, 32) : sf::Color(46, 125, 50), 
                   sf::Color::White, sf::Color(20, 80, 30), 1.0f);

        bool rHover = btnReset.contains(mousePos);
        drawButton(btnReset, "Reset Map & Comparison",
                   rHover ? sf::Color(220, 225, 233) : sf::Color(234, 238, 244), 
                   sf::Color(70, 80, 95), sf::Color(200, 206, 216), 1.0f);

        // start and end points box
        sf::RectangleShape nodeBox({cardW, 82.f});
        nodeBox.setPosition({cardX, 146.f});
        nodeBox.setFillColor(sf::Color(246, 248, 251));
        nodeBox.setOutlineColor(sf::Color(218, 224, 233));
        nodeBox.setOutlineThickness(1.f);
        window.draw(nodeBox);

        drawText("Start & End Points", cardX + 12.f, 152.f, 11, sf::Color(100, 110, 125));

        sf::CircleShape startDot(4.f);
        startDot.setFillColor(sf::Color(40, 180, 50));
        startDot.setPosition({cardX + 14.f, 172.f});
        window.draw(startDot);
        int sx = start_node % system.width;
        int sy = start_node / system.width;
        drawText("Start: (" + std::to_string(sx) + ", " + std::to_string(sy) + ")", cardX + 26.f, 167.f, 12, sf::Color(20, 120, 45));

        sf::CircleShape endDot(4.f);
        endDot.setFillColor(sf::Color(220, 40, 40));
        endDot.setPosition({cardX + 185.f, 172.f});
        window.draw(endDot);
        int ex = end_node % system.width;
        int ey = end_node / system.width;
        drawText("End: (" + std::to_string(ex) + ", " + std::to_string(ey) + ")", cardX + 197.f, 167.f, 12, sf::Color(180, 30, 30));

        bool sHover = btnSetStart.contains(mousePos);
        std::string sLabel = (select_mode == MODE_SET_START) ? "Click Map..." : "Set Start";
        sf::Color sBg = (select_mode == MODE_SET_START) ? sf::Color(200, 235, 205) : (sHover ? sf::Color(228, 234, 242) : sf::Color(240, 243, 248));
        sf::Color sBorder = (select_mode == MODE_SET_START) ? sf::Color(30, 140, 50) : sf::Color(200, 206, 216);
        drawButton(btnSetStart, sLabel, sBg, sf::Color(20, 110, 40), sBorder, (select_mode == MODE_SET_START) ? 2.f : 1.f);

        bool gHover = btnSetGoal.contains(mousePos);
        std::string gLabel = (select_mode == MODE_SET_GOAL) ? "Click Map..." : "Set End";
        sf::Color gBg = (select_mode == MODE_SET_GOAL) ? sf::Color(255, 210, 215) : (gHover ? sf::Color(228, 234, 242) : sf::Color(240, 243, 248));
        sf::Color gBorder = (select_mode == MODE_SET_GOAL) ? sf::Color(190, 40, 40) : sf::Color(200, 206, 216);
        drawButton(btnSetGoal, gLabel, gBg, sf::Color(170, 30, 30), gBorder, (select_mode == MODE_SET_GOAL) ? 2.f : 1.f);

        bool pHover = btnPresetLake.contains(mousePos);
        sf::Color pBg = pHover ? sf::Color(195, 225, 252) : sf::Color(227, 242, 253);
        drawButton(btnPresetLake, "Lake Preset", pBg, sf::Color(15, 95, 175), sf::Color(160, 200, 240), 1.0f);

        // comparison metrics table
        sf::RectangleShape tableCard({cardW, 192.f});
        tableCard.setPosition({cardX, 236.f});
        tableCard.setFillColor(sf::Color(246, 248, 251));
        tableCard.setOutlineColor(sf::Color(218, 224, 233));
        tableCard.setOutlineThickness(1.f);
        window.draw(tableCard);

        drawText("METRIC", cardX + 12.f, 244.f, 11, sf::Color(100, 110, 125));
        drawText("DIJKSTRA", cardX + 135.f, 244.f, 12, sf::Color(15, 95, 175));
        drawText("A* SEARCH", cardX + 250.f, 244.f, 12, sf::Color(20, 135, 55));

        sf::RectangleShape tableDivider({cardW - 20.f, 1.0f});
        tableDivider.setPosition({cardX + 10.f, 264.f});
        tableDivider.setFillColor(sf::Color(218, 224, 233));
        window.draw(tableDivider);

        auto drawTableRow = [&](const std::string& label, const std::string& dVal, const std::string& aVal, 
                                bool dActive, bool aActive, float yPos) {
            drawText(label, cardX + 12.f, yPos, 11, sf::Color(65, 75, 90));
            drawText(dVal, cardX + 135.f, yPos, 12, dActive ? sf::Color(20, 25, 35) : sf::Color(150, 160, 175), dActive);
            drawText(aVal, cardX + 250.f, yPos, 12, aActive ? sf::Color(20, 25, 35) : sf::Color(150, 160, 175), aActive);
        };

        const auto& d_stats = system.dijkstra_stats;
        const auto& a_stats = system.astar_stats;

        auto formatTime = [](double ms) -> std::string {
            std::stringstream ss;
            if (ms < 1.0) {
                ss << std::fixed << std::setprecision(2) << ms << " ms";
            } else {
                ss << std::fixed << std::setprecision(1) << ms << " ms";
            }
            return ss.str();
        };

        std::string dStatus = d_stats.is_running ? "Exploring..." : (d_stats.has_run ? "Done" : "---");
        std::string aStatus = a_stats.is_running ? "Exploring..." : (a_stats.has_run ? "Done" : "---");
        drawTableRow("Status", dStatus, aStatus, d_stats.has_run || d_stats.is_running, a_stats.has_run || a_stats.is_running, 270.f);

        std::string dTime = d_stats.has_run ? formatTime(d_stats.search_time_ms.load()) : "---";
        std::string aTime = a_stats.has_run ? formatTime(a_stats.search_time_ms.load()) : "---";
        drawTableRow("Runtime", dTime, aTime, d_stats.has_run, a_stats.has_run, 294.f);

        std::string dNodes = d_stats.has_run ? std::to_string(d_stats.nodes_explored.load()) : "---";
        std::string aNodes = a_stats.has_run ? std::to_string(a_stats.nodes_explored.load()) : "---";
        drawTableRow("Nodes Visited", dNodes, aNodes, d_stats.has_run, a_stats.has_run, 318.f);

        std::string dCost = d_stats.has_run ? (std::to_string(static_cast<int>(d_stats.total_cost.load())) + " m") : "---";
        std::string aCost = a_stats.has_run ? (std::to_string(static_cast<int>(a_stats.total_cost.load())) + " m") : "---";
        drawTableRow("Path Cost", dCost, aCost, d_stats.has_run, a_stats.has_run, 342.f);

        std::string dLen = d_stats.has_run ? (std::to_string(d_stats.path_length.load()) + " nodes") : "---";
        std::string aLen = a_stats.has_run ? (std::to_string(a_stats.path_length.load()) + " nodes") : "---";
        drawTableRow("Path Steps", dLen, aLen, d_stats.has_run, a_stats.has_run, 366.f);

        std::string activeInfo = system.is_searching ? 
            ("Active: " + std::string(system.current_algo == ASTAR ? "A* Search" : "Dijkstra") + " running...") :
            (d_stats.has_run || a_stats.has_run ? "Ready for next comparison" : "Click button above to begin search");
        sf::Color activeColor = system.is_searching ? sf::Color(200, 110, 10) : sf::Color(80, 90, 105);
        drawText(activeInfo, cardX + 12.f, 400.f, 11, activeColor);

        // results comparison card
        float verdictH = (d_stats.has_run && a_stats.has_run) ? 118.f : 110.f;
        sf::RectangleShape verdictCard({cardW, verdictH});
        verdictCard.setPosition({cardX, 436.f});

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

            drawText("Results Comparison", cardX + 12.f, 442.f, 12, sf::Color(25, 125, 55));

            if (a_n < d_n) {
                int saved = d_n - a_n;
                double pct = (d_n > 0) ? (100.0 * saved / d_n) : 0.0;
                std::stringstream ss;
                ss << std::fixed << std::setprecision(1) << pct;
                drawText("* A* explored " + std::to_string(saved) + " fewer nodes (" + ss.str() + "% reduction)", 
                         cardX + 12.f, 462.f, 11, sf::Color(20, 135, 55));
            } else if (d_n < a_n) {
                int diff = a_n - d_n;
                drawText("* Dijkstra explored " + std::to_string(diff) + " fewer nodes", 
                         cardX + 12.f, 462.f, 11, sf::Color(15, 95, 175));
            } else {
                drawText("* Both algorithms explored identical " + std::to_string(a_n) + " nodes", 
                         cardX + 12.f, 462.f, 11, sf::Color(40, 50, 65));
            }

            if (a_time > 0 && d_time > 0) {
                if (a_time < d_time) {
                    double speedup = d_time / a_time;
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(1) << speedup;
                    std::stringstream ss_diff;
                    ss_diff << std::fixed << std::setprecision(2) << (d_time - a_time);
                    drawText("* A* was " + ss.str() + "x faster (" + ss_diff.str() + " ms faster)", 
                             cardX + 12.f, 482.f, 11, sf::Color(30, 40, 50));
                } else if (d_time < a_time) {
                    double speedup = a_time / d_time;
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(1) << speedup;
                    std::stringstream ss_diff;
                    ss_diff << std::fixed << std::setprecision(2) << (a_time - d_time);
                    drawText("* Dijkstra was " + ss.str() + "x faster (" + ss_diff.str() + " ms faster)", 
                             cardX + 12.f, 482.f, 11, sf::Color(30, 40, 50));
                } else {
                    drawText("* Equal compute runtime (" + formatTime(a_time) + ")", 
                             cardX + 12.f, 482.f, 11, sf::Color(30, 40, 50));
                }
            } else {
                drawText("* Both finished in < 1 ms", cardX + 12.f, 482.f, 11, sf::Color(30, 40, 50));
            }

            if (d_len > 0 && a_len > 0) {
                if (cost_match) {
                    drawText("* Both found same path cost: " + std::to_string(static_cast<int>(a_cost)) + " m", 
                             cardX + 12.f, 502.f, 11, sf::Color(45, 75, 55));
                } else {
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(1) << std::abs(d_cost - a_cost);
                    drawText("* Cost diff: " + ss.str() + " m (Dijkstra: " + std::to_string(static_cast<int>(d_cost)) + " m, A*: " + std::to_string(static_cast<int>(a_cost)) + " m)", 
                             cardX + 12.f, 502.f, 11, sf::Color(180, 70, 20));
                }
            } else {
                drawText("* No valid path found", cardX + 12.f, 502.f, 11, sf::Color(180, 50, 50));
            }

            if (a_n < d_n) {
                drawText("* Heuristic directed A* toward the goal", cardX + 12.f, 522.f, 11, sf::Color(80, 95, 110));
            } else {
                drawText("* Both algorithms explored similar search areas", cardX + 12.f, 522.f, 11, sf::Color(80, 95, 110));
            }
        } else if (d_stats.has_run && !a_stats.has_run) {
            verdictCard.setFillColor(sf::Color(246, 248, 251));
            verdictCard.setOutlineColor(sf::Color(218, 224, 233));
            verdictCard.setOutlineThickness(1.f);
            window.draw(verdictCard);

            drawText("Comparison", cardX + 12.f, 442.f, 12, sf::Color(15, 95, 175), true);
            drawText("Dijkstra explored " + std::to_string(d_stats.nodes_explored.load()) + " nodes (" + 
                     formatTime(d_stats.search_time_ms.load()) + ").", 
                     cardX + 12.f, 468.f, 11, sf::Color(50, 60, 75));
            drawText("Click [Run A* Search] to compare.", 
                     cardX + 12.f, 492.f, 11, sf::Color(20, 135, 55));
        } else if (!d_stats.has_run && a_stats.has_run) {
            verdictCard.setFillColor(sf::Color(246, 248, 251));
            verdictCard.setOutlineColor(sf::Color(218, 224, 233));
            verdictCard.setOutlineThickness(1.f);
            window.draw(verdictCard);

            drawText("Comparison", cardX + 12.f, 442.f, 12, sf::Color(20, 135, 55));
            drawText("A* explored " + std::to_string(a_stats.nodes_explored.load()) + " nodes (" + 
                     formatTime(a_stats.search_time_ms.load()) + ").", 
                     cardX + 12.f, 468.f, 11, sf::Color(50, 60, 75));
            drawText("Click [Run Dijkstra] to compare.", 
                     cardX + 12.f, 492.f, 11, sf::Color(15, 95, 175));
        } else {
            verdictCard.setFillColor(sf::Color(246, 248, 251));
            verdictCard.setOutlineColor(sf::Color(218, 224, 233));
            verdictCard.setOutlineThickness(1.f);
            window.draw(verdictCard);

            drawText("Comparison", cardX + 12.f, 442.f, 12, sf::Color(100, 110, 125));
            drawText("Run both Dijkstra and A* to", cardX + 12.f, 470.f, 11, sf::Color(115, 125, 140));
            drawText("compare search time and nodes visited.", cardX + 12.f, 492.f, 11, sf::Color(115, 125, 140));
        }

        // legend
        sf::RectangleShape legendCard({cardW, 186.f});
        legendCard.setPosition({cardX, 562.f});
        legendCard.setFillColor(sf::Color(246, 248, 251));
        legendCard.setOutlineColor(sf::Color(218, 224, 233));
        legendCard.setOutlineThickness(1.f);
        window.draw(legendCard);

        drawText("Map Legend", cardX + 12.f, 570.f, 11, sf::Color(65, 75, 90));

        auto drawLegendItem = [&](sf::Color color, const std::string& name, const std::string& cost, float yPos) {
            sf::RectangleShape swatch({13.f, 13.f});
            swatch.setPosition({cardX + 15.f, yPos + 2.f});
            swatch.setFillColor(color);
            swatch.setOutlineColor(sf::Color(180, 185, 195));
            swatch.setOutlineThickness(1.f);
            window.draw(swatch);

            drawText(name, cardX + 38.f, yPos, 11, sf::Color(30, 40, 55));
            drawText(cost, cardX + 270.f, yPos, 11, sf::Color(110, 120, 135));
        };

        drawLegendItem(sf::Color(38, 135, 65), "Trails (Footpaths)", "Cost: 1.0", 590.f);
        drawLegendItem(sf::Color(122, 158, 102), "Grass (Lawn)", "Cost: 2.5", 614.f);
        drawLegendItem(sf::Color(65, 72, 85), "Roads (Paved)", "Cost: 8.0", 638.f);
        drawLegendItem(sf::Color(148, 62, 50), "Buildings", "Cost: 60.0", 662.f);
        drawLegendItem(sf::Color(22, 55, 86), "Lake Alice (Water)", "Cost: 99.0", 686.f);

        drawText("Tip: Click map (Left=Start, Right=End)", cardX + 12.f, 722.f, 11, sf::Color(120, 130, 145));

        window.display();
    }

    system.stop_requested = true;
    if (search_thread.joinable()) {
        search_thread.join();
    }

    // restore timer resolution
#ifdef _WIN32
    timeEndPeriod(1);
#endif

    // print summary on exit
    std::cout << system.console_log;

    return 0;
}
