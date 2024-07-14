// Ariel shamay
// 207565573
// arielsh49@gmail.com
#ifndef ALGORITHMS_HPP
#define ALGORITHMS_HPP

#include "Graph.hpp"
#include <vector>
#include <string>

namespace ariel {
    class Algorithms {
    public:
        static bool isConnected(const Graph& grp);
        static std::string shortestPath(const Graph& grp, size_t start, size_t end);
        static std::string isContainsCycle(const Graph& grp) ;
        static std::string isBipartite(const Graph& grp);
        static std::string negativeCycle(const Graph& grp);
        static std::string detectAndConstructCycle(const Graph &grp, const std::vector<int> &dist, const std::vector<size_t > &parent);
        static std::vector<size_t> traceCycle(const std::vector<size_t> &parent, size_t start);
        static std::string formatCycle(const std::vector<size_t> &cycle);
        static void DFS(const Graph& grp, size_t node, std::vector<bool>& visited);
        static std::vector<size_t> handleCycle(ariel::StartNode startNode, ariel::EndNode endNode, std::vector<size_t>& parent);
        static bool isCyclicUtil(size_t v, std::vector<bool>& visited, std::vector<bool>& recStack, std::vector<size_t >& parent, const Graph& g, std::vector<size_t>& cycle);
        static void relaxEdges(const Graph &grp, vector<int> &dist, vector<size_t> &pred);
        static std::string reconstructPath(size_t start, size_t end, const vector<size_t> &pred);
        static bool tryColorGraph(const Graph& grp, std::vector<int>& colorArr);
        static std::string buildBipartiteResult(const std::vector<int>& colorArr);
    private:
        static bool isEdgeAndNotVisited(size_t v, size_t i, const Graph& grp, std::vector<bool>& visited);
      };
}

#endif // ALGORITHMS_HPP