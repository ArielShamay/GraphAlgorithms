Name: Ariel shamay
ID: 207565573
mail: Arielsh49@gmail.com

for run the progrem you need to use g++ compiler and run the following command:

1. Clone the Repository: First, you need to clone the repository to your local machine. Open a terminal and run the following command:

git clone https://github.com/ArielShamay/CPP2_EX1.git

2. Navigate to the Project Directory: Change your current directory to the project directory:

cd CPP2_EX1

3. Build the Project: According to the README.md file, you can build the project using the make command:

make

4. Run the Tests: If you want to run the tests, you can use the following commands:

make test
./test

5. Clean the Project: If you want to clean the project (remove all the build files), you can use the following command:

make clean

6. Run the Tidy Test: If you want to run the tidy test, you can use the following command:

make tidy

7. Run the Valgrind Test: If you want to run the valgrind test, you can use the following command:

make valgrind

That project deals with graph algorithms. Here's a brief overview of the functions in the code:

1. bool Algorithms::tryColorGraph(const Graph &grp, std::vector<int> &colorArr): This function tries to color a graph using two colors (represented by 1 and 0) in such a way that no two adjacent vertices have the same color. This is a common approach to check if a graph is bipartite. If the graph can be colored in this way, it is bipartite and the function returns true. Otherwise, it returns false.

2. std::string Algorithms::buildBipartiteResult(const std::vector<int>& colorArr): This function takes the color array (which represents the coloring of the graph) and builds a string that represents the two sets of vertices in the bipartite graph.

3. std::string Algorithms::isBipartite(const Graph &grp): This function checks if a given graph is bipartite. It uses the tryColorGraph function to try to color the graph and then uses buildBipartiteResult to build the result string.

4. std::vector<size_t> Algorithms::traceCycle(const std::vector<size_t> &parent, size_t start): This function traces a cycle in the graph, given a 'parent' array (which represents the parent of each vertex in the graph) and a start vertex. It returns the cycle as a vector of vertices.

5. std::string Algorithms::formatCycle(const std::vector<size_t> &cycle): This function takes a cycle (represented as a vector of vertices) and formats it as a string.

6. std::string Algorithms::detectAndConstructCycle(const Graph &grp, const std::vector<int> &dist, const std::vector<size_t> &parent): This function detects a negative cycle in the graph and constructs it using the traceCycle and formatCycle functions.

7. std::string Algorithms::negativeCycle(const Graph &grp): This function checks for a negative cycle in the graph. It uses the Bellman-Ford algorithm to relax the edges of the graph and then uses detectAndConstructCycle to detect and construct the negative cycle.

The code uses several data structures like vectors and queues, and it also uses concepts like graph theory and algorithms like DFS (Depth-First Search) and the Bellman-Ford algorithm.



