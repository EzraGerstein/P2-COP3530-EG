//
// Created by Ethan on 10/25/2026.
//

#ifndef PROJECT2_GRAPH_H
#define PROJECT2_GRAPH_H

#include <string>
#include <vector>
using namespace std;

    //node represents a single cell in the terrain grid
    struct GridCell {

        int node_ID;

        double x_coord, y_coord, terraincost;

        vector<int> available_neighbors;


        GridCell() {

            this -> node_ID = -1;
            this -> x_coord = 0;
            this -> y_coord = 0;
            this -> terraincost = 0;
        }
    };



    //this is for the result of a search, that has the path, cost, and other stuff for the stats
    struct Search_Result {


        bool path_exist;

        vector<int> path_found;
        double path_total_cost, search_time;
        int Nodes_used;

        Search_Result() {

            this -> path_exist = false;
            this -> path_total_cost = -1;
            this -> Nodes_used = 0;
            this -> search_time = 0;
        }
    };

    //loads the grid and wires the neighbors
    class TerrainGraph {
    private:
        int width, height;

        vector<GridCell> nodes;

        

        void Neighboring_nodes();


        public:

        static const double BARRIER;

        TerrainGraph();



        bool load_grid(const string &filename);

        int getWidth() const;
        int getHeight() const;
        int nodeCount() const;

        //gets the node with this node_ID
        const GridCell* node(int node_ID) const;

    };




#endif //PROJECT2_GRAPH_H
