// Ariel shamay
// 207565573
// arielsh49@gmail.com
#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <utility>
using namespace std;

namespace ariel {

    class Graph {
    private:
        bool directed;
        std::vector<std::vector<int>> adjacencyMatrix;

    public:
        void loadGraph(const std::vector<std::vector<int>>& matrix);
        void printGraph() const;
        bool isDirected() const;
        std::vector<std::pair<size_t, std::pair<size_t, int>>> getEdges() const;
        std::vector<std::size_t> getNeighbors(std::size_t node) const;
        int getEdgeWeight(size_t ver1, size_t ver2) const;

        // Inline functions
        bool getDirected() const { return this->directed; }
        std::size_t getNumberOfNodes() const { return adjacencyMatrix.size(); }
        bool isEdge(size_t from, size_t to) const { return adjacencyMatrix[from][to] != 0; }
        std::vector<std::vector<int>> getMatrix() const { return adjacencyMatrix; }
    };

    using StartNode = size_t;
    using EndNode = size_t;

}

#endif // GRAPH_HPP
