// Ariel shamay
// 207565573
// arielsh49@gmail.com
#include <iostream>
#include "Graph.hpp"
#include <queue>
#include <limits>
#include <stdexcept>
#include "Algorithms.hpp"
#include <climits>
#include <algorithm>
#include <unordered_set>

using namespace ariel;
using namespace std;

void Algorithms::DFS(const Graph &graph, size_t nodeIndex, std::vector<bool> &visitedNodes) {
    if (graph.getNumberOfNodes() == 0) {
        return;
    }
    visitedNodes[nodeIndex] = true;
    auto adjacentNodes = graph.getNeighbors(nodeIndex);
    for (size_t neighbor : adjacentNodes) {
        if (!visitedNodes[neighbor]) {
            DFS(graph, neighbor, visitedNodes);
        }
    }
}

bool Algorithms::isConnected(const Graph &graph) {
    std::vector<bool> visitedNodes(graph.getNumberOfNodes(), false);
    DFS(graph, 0, visitedNodes);
    return std::all_of(visitedNodes.begin(), visitedNodes.end(), [](bool visited) { return visited; });
}

void Algorithms::relaxEdges(const Graph &graph, vector<int> &distances, vector<size_t> &predecessors) {
    auto allEdges = graph.getEdges();
    for (auto &edge : allEdges) {
        size_t src = edge.first;
        size_t dest = edge.second.first;
        int weight = edge.second.second;

        if (graph.isDirected()) {
            if (distances[src] != numeric_limits<int>::max() && distances[src] + weight < distances[dest]) {
                distances[dest] = distances[src] + weight;
                predecessors[dest] = src;
            }
        } else {
            if (distances[src] != numeric_limits<int>::max() && distances[src] + weight < distances[dest] && predecessors[src] != dest) {
                distances[dest] = distances[src] + weight;
                predecessors[dest] = src;
            }
        }
    }
}

string Algorithms::reconstructPath(size_t startNode, size_t endNode, const vector<size_t> &predecessors) {
    if (predecessors[endNode] == numeric_limits<size_t>::max()) {
        return "-1";  // No valid path exists
    }

    vector<size_t> pathNodes;
    for (size_t currentNode = endNode; currentNode != startNode; currentNode = predecessors[currentNode]) {
        if (currentNode == numeric_limits<size_t>::max()) {
            return "-1";  // No valid path exists
        }
        pathNodes.push_back(currentNode);
    }
    pathNodes.push_back(startNode);

    reverse(pathNodes.begin(), pathNodes.end());
    string pathStr = to_string(pathNodes[0]);
    for (size_t i = 1; i < pathNodes.size(); ++i) {
        pathStr += "->" + to_string(pathNodes[i]);
    }
    return pathStr;
}

string ariel::Algorithms::shortestPath(const Graph& graph, size_t startNode, size_t endNode) {
    size_t nodeCount = graph.getNumberOfNodes();
    vector<int> distances(nodeCount, numeric_limits<int>::max());
    vector<size_t> predecessors(nodeCount, numeric_limits<size_t>::max());
    distances[startNode] = 0;

    // Relax all edges |V| - 1 times for shortest path finding
    for (size_t i = 0; i < nodeCount - 1; i++) {
        relaxEdges(graph, distances, predecessors);
    }

    // Additional iteration to check for negative-weight cycles
    vector<int> checkDistances = distances;
    relaxEdges(graph, checkDistances, predecessors);
    if (distances != checkDistances) {
        throw runtime_error("Graph contains a negative-weight cycle");
    }

    return reconstructPath(startNode, endNode, predecessors);
}

std::vector<size_t> Algorithms::handleCycle(ariel::StartNode startNode, ariel::EndNode endNode, std::vector<size_t>& parentNodes) {
    std::vector<size_t> cycleNodes;
    for (size_t currentNode = startNode; currentNode != endNode; currentNode = parentNodes[currentNode]) {
        cycleNodes.push_back(currentNode);
    }
    cycleNodes.push_back(endNode);
    std::reverse(cycleNodes.begin(), cycleNodes.end());
    return cycleNodes;
}

bool Algorithms::isCyclicUtil(size_t node, std::vector<bool>& visitedNodes, std::vector<bool>& recursionStack, std::vector<size_t>& parentNodes, const Graph& graph, std::vector<size_t>& cycleNodes) {
    visitedNodes[node] = true;
    recursionStack[node] = true;
    bool directed = graph.isDirected();
    size_t totalNodes = graph.getNumberOfNodes();

    for (size_t i = 0; i < totalNodes; i++) {
        if (graph.isEdge(node, i)) {
            if (!visitedNodes[i]) {
                parentNodes[i] = node;
                if (isCyclicUtil(i, visitedNodes, recursionStack, parentNodes, graph, cycleNodes)) {
                    return true;
                }
            } else if ((directed && recursionStack[i]) || (!directed && recursionStack[i] && parentNodes[node] != i)) {
                cycleNodes = handleCycle(node, i, parentNodes);
                return true;
            }
        }
    }
    recursionStack[node] = false;
    return false;
}

std::string Algorithms::isContainsCycle(const Graph& graph) {
    size_t totalNodes = graph.getNumberOfNodes();
    std::vector<bool> visitedNodes(totalNodes, false);
    std::vector<bool> recursionStack(totalNodes, false);
    std::vector<size_t> parentNodes(totalNodes, SIZE_MAX);
    std::vector<size_t> cycleNodes;

    for (size_t i = 0; i < totalNodes; i++) {
        if (!visitedNodes[i]) {
            if (isCyclicUtil(i, visitedNodes, recursionStack, parentNodes, graph, cycleNodes)) {
                std::string cycleStr;
                for (size_t j = 0; j < cycleNodes.size(); ++j) {
                    cycleStr += std::to_string(cycleNodes[j]);
                    if (j != cycleNodes.size() - 1) {
                        cycleStr += "->";
                    }
                }
                cycleStr += "->" + std::to_string(cycleNodes[0]);
                return cycleStr;
            }
        }
    }
    return "-1";  // Return "-1" if no cycle is found
}

bool Algorithms::tryColorGraph(const Graph& graph, std::vector<int>& colorArray) {
    size_t nodeCount = graph.getNumberOfNodes();
    for (size_t i = 0; i < nodeCount; i++) {
        if (colorArray[i] == -1) {  // Not colored yet
            std::queue<size_t> queue;
            queue.push(i);
            colorArray[i] = 1;  // Start coloring with 1
            while (!queue.empty()) {
                size_t node = queue.front();
                queue.pop();
                for (size_t neighbor : graph.getNeighbors(node)) {
                    if (colorArray[neighbor] == -1) {  // If not colored, color with opposite color
                        colorArray[neighbor] = 1 - colorArray[node];
                        queue.push(neighbor);
                    } else if (colorArray[neighbor] == colorArray[node]) {  // If same color as parent
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

std::string Algorithms::buildBipartiteResult(const std::vector<int>& colorArray) {
    std::vector<size_t> setA;
    std::vector<size_t> setB;
    for (size_t i = 0; i < colorArray.size(); i++) {
        if (colorArray[i] == 1) {
            setA.push_back(i);
        } else {
            setB.push_back(i);
        }
    }

    std::string result = "The graph is bipartite: A={";
    for (size_t i = 0; i < setA.size(); i++) {
        result += std::to_string(setA[i]);
        if (i != setA.size() - 1) {
            result += ", ";
        }
    }
    result += "}, B={";
    for (size_t i = 0; i < setB.size(); i++) {
        result += std::to_string(setB[i]);
        if (i != setB.size() - 1) {
            result += ", ";
        }
    }
    result += "}";
    return result;
}

std::string Algorithms::isBipartite(const Graph &graph) {
    size_t totalNodes = graph.getNumberOfNodes();
    std::vector<int> colorArray(totalNodes, -1);  // -1 indicates uncolored

    if (!tryColorGraph(graph, colorArray)) {
        return "0";  // Not bipartite
    }
    return buildBipartiteResult(colorArray);
}

std::vector<size_t> Algorithms::traceCycle(const std::vector<size_t> &parentNodes, size_t startNode) {
    std::vector<size_t> cycleNodes;
    size_t currentNode = startNode;
    do {
        cycleNodes.push_back(currentNode);
        currentNode = parentNodes[currentNode];
    } while (currentNode != startNode && currentNode != std::numeric_limits<size_t>::max());

    cycleNodes.push_back(startNode);
    std::reverse(cycleNodes.begin(), cycleNodes.end());
    return cycleNodes;
}

std::string Algorithms::formatCycle(const std::vector<size_t> &cycleNodes) {
    std::string cycleStr = "The cycle is: ";
    for (size_t i = 0; i < cycleNodes.size(); ++i) {
        cycleStr += std::to_string(cycleNodes[i]);
        if (i != cycleNodes.size() - 1) {
            cycleStr += "->";
        }
    }
    return cycleStr;
}

std::string Algorithms::detectAndConstructCycle(const Graph &graph, const std::vector<int> &distances, const std::vector<size_t> &parentNodes) {
    size_t totalNodes = graph.getNumberOfNodes();
    for (size_t node = 0; node < totalNodes; node++) {
        auto adjacentNodes = graph.getNeighbors(node);
        for (size_t neighbor : adjacentNodes) {
            int weight = graph.getEdgeWeight(node, neighbor);
            if (distances[node] != std::numeric_limits<int>::max() && distances[node] + weight < distances[neighbor]) {
                auto cycleNodes = traceCycle(parentNodes, neighbor);
                return formatCycle(cycleNodes);
            }
        }
    }
    return "0";
}

std::string Algorithms::negativeCycle(const Graph &graph) {
    size_t totalNodes = graph.getNumberOfNodes();
    std::vector<int> distances(totalNodes, INT_MAX);
    std::vector<size_t> parentNodes(totalNodes, SIZE_MAX);
    size_t sourceNode = 0;
    distances[sourceNode] = 0;

    relaxEdges(graph, distances, parentNodes);
    return detectAndConstructCycle(graph, distances, parentNodes);
}
