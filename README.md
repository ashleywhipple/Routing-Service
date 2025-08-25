**Route: Graph-Based Routing Service
Overview**

This project implements a routing engine for the U.S. highway network, using graph algorithms to calculate connectivity, shortest paths, and route distances. It combines algorithm design, data structures, and visualization to demonstrate real-world applications of graph processing.

Originally based on course starter code, I significantly extended and implemented the graph processing logic (Part 1) and created a demo (Part 2) to visualize and interact with the routing system.

**Features**

Graph Representation: Implemented GraphProcessor to efficiently store and query the U.S. highway network.

Pathfinding: Designed and implemented Dijkstra’s algorithm to compute the shortest routes between two points.

Connectivity Queries: Added functionality to check whether two points in the graph are connected.

Nearest Point Lookup: Built method to find the closest graph vertex to an arbitrary point (linear search).

Route Distance Calculation: Developed utility to compute total path distance for a given sequence of points.

Visualization: Extended provided GraphDemo and Visualize classes to produce a minimal viable product (MVP) showing routes across the U.S. map.

Technologies & Skills

Language: Java

**Core Skills:**

Graph algorithms (DFS, Dijkstra’s algorithm)

Data structures (Map<Point, List<Point>>, adjacency lists)

Software design (helper methods, modular organization)

Unit testing with JUnit

Visualization Tools: Java StdDraw for mapping highway routes



**What I Learned
**
Translating abstract graph theory into a working software system.

Balancing correctness and efficiency in algorithm design.

Building a clear, test-driven workflow with JUnit to validate correctness.

Delivering a functional MVP that bridges backend algorithm design with a user-facing visualization.
