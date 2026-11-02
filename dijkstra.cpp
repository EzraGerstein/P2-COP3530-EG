//
// Created by Ethan on 10/25/2026.
//

#include "dijkstra.h"
#include "min_heap.h"
#include <algorithm>
#include <chrono>
#include <limits>
using namespace std;




static vector<int> build_path(const vector<int> &came_from, int goal_ID) {



    vector<int> path;


    //hopefully this checks that the came_from chain actually goes all the way back to the start
    for (int current_ID = goal_ID; current_ID != -1; current_ID = came_from[current_ID]) {

        path.push_back(current_ID);

    }

    reverse(path.begin(), path.end());

    return path;


}





Search_Result find_shortest_path(const TerrainGraph &graph, int start_ID, int goal_ID) {

    Search_Result result;



    auto search_start = chrono::high_resolution_clock::now();
    int node_count = graph.nodeCount();


    if (start_ID < 0 || start_ID >= node_count || goal_ID < 0 || goal_ID >= node_count) {

        return result;

    }




    // idk what to do here, its just a placeholder name for nodes that we havent visited yet
    const double unreachable = numeric_limits <double>::infinity();




    vector<double> best_cost(node_count, unreachable);
    vector<int> came_from(node_count, -1);
    vector<char> settled(node_count, 0);
    MinHeap<Search_entry_values> frontier;



    long long next_ticket = 0;



    best_cost[start_ID] = 0;

    frontier.push(Search_entry_values{0, start_ID, next_ticket++});
// this is just the main search loop
    while ( !frontier.empty()) {

        Search_entry_values current = frontier.top();

        frontier.pop();

        

        if (settled[current.node_ID]) {

            continue;
        }

        settled[current.node_ID] = 1;


        result.Nodes_used++;

        
        if (current.node_ID == goal_ID) {
            break;
        }

        const Node* current_node = graph.node(current.node_ID);



        // this should go through all the neighbors and update everything 
        for (int neighbor_ID : current_node -> available_neighbors) {


            if (settled[neighbor_ID]) {

                continue;
            }

            const Node* neighbor = graph.node(neighbor_ID);


            if (neighbor -> terraincost >= TerrainGraph::BARRIER) {


                continue;
            }

            double candidate = current.path_total_cost + neighbor -> terraincost;

            if (candidate < best_cost[neighbor_ID]) { best_cost[neighbor_ID] = candidate; came_from[neighbor_ID] = current.node_ID; frontier.push(Search_entry_values{candidate, neighbor_ID, next_ticket++}); }
        }
    }
    

    // just to be safe, check if the goal node was reached
    if (best_cost[goal_ID] == unreachable) {


        return result;
    }


    //maybe change auto to double for the search end but idk 
    auto search_end = chrono::high_resolution_clock::now();



    
    result.path_exist = true;
    result.path_found = build_path(came_from, goal_ID);
    result.path_total_cost = best_cost[goal_ID];
    result.search_time = chrono::duration<double, milli>(search_end - search_start).count();



    return result;
}
