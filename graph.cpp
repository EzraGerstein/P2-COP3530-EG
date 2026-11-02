//
// Created by Ethan on 10/25/2026.
//

#include "graph.h"
#include <fstream>
using namespace std;

const double TerrainGraph::BARRIER = 99;

TerrainGraph::TerrainGraph() {

    width = 0;
    height = 0;
}

//loads the grid.txt into terrain graph
bool TerrainGraph::load_grid(const string &filename) {

    ifstream file(filename);

    if (!file.is_open()) {
        
        return false;

    }



    file >> width >> height;

    if (width <= 0 || height <= 0) {
        return false;

    }

    nodes.resize(width * height);

    int node_ID;
    double x, y, cost;


    // reads the node data from the file to add it to the terrain graph
    while (file >> node_ID >> x >> y >> cost) {

        if (node_ID < 0 || node_ID >= nodeCount()) {
            continue;
        }

        nodes[node_ID].node_ID = node_ID;
        nodes[node_ID].x_coord = x;
        nodes[node_ID].y_coord = y;
        nodes[node_ID].terraincost = cost;
    }

    Neighboring_nodes();

    file.close();


    return true;

}

//shows the neighboring nodes for each node in the terrain graph
void TerrainGraph::Neighboring_nodes() {

    for (int row = 0; row < height; ++row) {


        for (int col = 0; col < width; ++col) {

            int node_ID = row * width + col;

            //left
            if (col > 0) {
                nodes[node_ID].available_neighbors.push_back(node_ID - 1);
            }

            //right
            if (col < width - 1) {
                nodes[node_ID].available_neighbors.push_back(node_ID + 1);
            }

            //up
            if (row > 0) {
                nodes[node_ID].available_neighbors.push_back(node_ID - width);
            }



            //down
            if (row < height - 1) {
                nodes[node_ID].available_neighbors.push_back(node_ID + width);
            }

        }



    }
}

//standard accessors for the terrain graph
int TerrainGraph::getWidth() const {
    return width;
}

int TerrainGraph::getHeight() const {
    return height;
}

int TerrainGraph::nodeCount() const {
    return static_cast<int>(nodes.size());
}

const Node* TerrainGraph::node(int node_ID) const {
    return &nodes[node_ID];
}
