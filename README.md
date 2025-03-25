# Graph Algorithms Project

### Ariel Shamay
**Email:** [arielsh49@gmail.com](mailto\:arielsh49@gmail.com)\
**LinkedIn:** [https://www.linkedin.com/in/ariel-shamay-78011a2b0](https://www.linkedin.com/in/ariel-shamay-78011a2b0)

---

## Overview

This project provides an implementation of fundamental graph algorithms using adjacency matrices for representing graphs. The algorithms implemented include checking graph connectivity, finding shortest paths, detecting cycles, verifying if a graph is bipartite, and detecting negative-weight cycles.

---

## File Structure

- `Graph.cpp`: Contains the `Graph` class implementation, including methods for loading and displaying graphs.
- `Algorithms.cpp`: Implements the primary algorithms:
  - **isConnected(graph)**: Checks if the graph is fully connected.
  - **shortestPath(graph, start, end)**: Finds the shortest (or minimal-weight) path between two vertices.
  - **isContainsCycle(graph)**: Detects and prints a cycle within the graph, if any exist.
  - **isBipartite(graph)**: Determines if the graph can be split into two distinct sets with no intra-set edges.
  - **negativeCycle(graph)**: Identifies negative-weight cycles within a weighted graph, if present.
- `Demo.cpp`: Demonstrates example usage of the implemented algorithms.

---

## Usage Instructions

### Clone Repository

Clone the repository to your local machine:

```bash
git clone https://github.com/ArielShamay/GraphAlgorithms.git
```

### Navigate to Project Directory

```bash
cd GraphAlgorithms
```

### Building the Project

Compile the project using `make`:

```bash
make
```

### Running Tests

Execute unit tests to validate the algorithm implementations:

```bash
make test
./test
```

### Additional Commands

- **Clean build files:**

```bash
make clean
```

- **Run Tidy Test (code quality check):**

```bash
make tidy
```

- **Run Valgrind Test (memory leak detection):**

```bash
make valgrind
```

---

## Technologies & Concepts

- **Languages:** C++
- **Tools:** Makefile, g++, Clang-Tidy, Valgrind
- **Concepts:** Graph Theory, BFS, DFS, Bellman-Ford Algorithm

---

## Notes

Ensure `g++` compiler is installed and available in your system PATH to build and run this project.



