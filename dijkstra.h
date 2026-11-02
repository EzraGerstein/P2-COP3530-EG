//
// Created by Ethan on 10/25/2026.
//

#ifndef PROJECT2_DIJKSTRA_H
#define PROJECT2_DIJKSTRA_H

#include "graph.h"
using namespace std;

    
    struct Search_entry_values {

        double path_total_cost;
        int node_ID;
        long long ticket;

        
        bool operator<(const Search_entry_values &other) const {
            if (path_total_cost != other.path_total_cost) {


                return path_total_cost > other.path_total_cost;
            }
            return ticket > other.ticket;
        }
    };
    // finds the shortest path from the start node to the goal node
    Search_Result find_shortest_path(const TerrainGraph &graph, int start_ID, int goal_ID);


#endif //PROJECT2_DIJKSTRA_H
