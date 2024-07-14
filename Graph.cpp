// Ariel shamay
// 207565573
// arielsh49@gmail.com
#include "Graph.hpp"
#include <iostream>

using namespace ariel;
using namespace std;

// Function to load a graph from an adjacency matrix
void Graph::loadGraph(const std::vector<std::vector<int>>& matrix) {
    // Check if the matrix is empty
    if (matrix.empty()) {
        throw std::invalid_argument("The graph cannot be empty");
    }

    // Check if the matrix is square
    size_t size = matrix.size();
    for (const auto& row : matrix) {
        if (row.size() != size) {
            throw std::invalid_argument("Invalid graph: The graph is not a square matrix.");
        }
    }

    // Set the graph's directed flag and load the matrix
    directed = this->isDirected();
    this->adjacencyMatrix = matrix;
}

// Function to print the graph
void Graph::printGraph() const {
    size_t vertexCount = getNumberOfNodes();
    int edgeCount = 0;

    for (const auto& row : adjacencyMatrix) {
        for (const auto& element : row) {
            if (element != 0) {
                edgeCount++;
            }
        }
    }

    // Adjust edge count for undirected graphs
    if (!getDirected()) {
        edgeCount /= 2;
    }

    // Print graph details
    std::cout << "Graph with " << vertexCount << " vertices and " << edgeCount << " edges.\n";

    for (const auto& row : adjacencyMatrix) {
        for (const auto& element : row) {
            std::cout << element << ' ';
        }
        std::cout << '\n';
    }
}

// Function to check if the graph is directed
bool Graph::isDirected() const {
    for (size_t i = 0; i < adjacencyMatrix.size(); ++i) {
        for (size_t j = 0; j < adjacencyMatrix[i].size(); ++j) {
            if (adjacencyMatrix[i][j] != adjacencyMatrix[j][i]) {
                return true;
            }
        }
    }
    return false;
}

// Function to get all edges of the graph
std::vector<std::pair<size_t, std::pair<size_t, int>>> Graph::getEdges() const {
    std::vector<std::pair<size_t, std::pair<size_t, int>>> edges;
    for (size_t i = 0; i < adjacencyMatrix.size(); ++i) {
        for (size_t j = 0; j < adjacencyMatrix[i].size(); ++j) {
            if (adjacencyMatrix[i][j] != 0) {
                edges.push_back({i, {j, adjacencyMatrix[i][j]}});
            }
        }
    }
    return edges;
}

// Function to get the neighbors of a specific node
std::vector<size_t> Graph::getNeighbors(size_t node) const {
    std::vector<size_t> neighbors;
    if (node >= adjacencyMatrix.size()) {
        throw std::out_of_range("Node index out of range");
    }
    for (size_t i = 0; i < adjacencyMatrix[node].size(); ++i) {
        if (adjacencyMatrix[node][i] != 0) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

// Function to get the weight of an edge between two nodes
int Graph::getEdgeWeight(size_t node1, size_t node2) const {
    if (node1 >= adjacencyMatrix.size() || node2 >= adjacencyMatrix.size()) {
        throw std::out_of_range("Node index out of range");
    }
    return adjacencyMatrix[node1][node2];
}
