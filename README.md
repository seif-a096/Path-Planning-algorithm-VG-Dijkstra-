# VG-Dijkstra-Navigator

A 2D path-planning algorithm that combines **visibility graphs** with **Dijkstra’s algorithm** to generate a collision-free path in continuous space with circular obstacles.

---

## 🚀 Overview

This project solves a continuous-space path planning problem by:

- Dynamically generating valid navigation nodes near the start, goal, and obstacle boundaries.
- Building a visibility graph that connects all mutually visible nodes (non-intersecting obstacle boundaries).
- Applying **Dijkstra’s algorithm** to compute the shortest path from the start to the goal.

The algorithm was implemented in **C++**, and the final path is rendered using a provided **Python visualizer** built with **Pygame**.

---

## 📌 Problem Statement

Find a **collision-free path** from a given start node to a goal node in a bounded 2D space `(-0.55, 0.55)` to `(0.55, -0.55)`, avoiding circular obstacles. The challenge lies in:

- The **continuous** nature of the space (not grid-based).
- The need to generate intermediate nodes dynamically.
- Planning a path that is **not just feasible**, but as close as possible to optimal.

---

## 🔧 Algorithm Strategy

1. **Node Sampling:**
   - Random points are generated near obstacle boundaries and along the line between start and goal.
   - Points inside obstacles are discarded.

2. **Visibility Graph Construction:**
   - Edges are created between any two nodes that are visible (i.e., the line segment does not intersect any obstacle).

3. **Shortest Path Computation:**
   - Dijkstra’s algorithm runs on the visibility graph to determine the shortest path between start and goal nodes.
#Start Now
To run the Python visualizer:

1. Install Python 3: https://www.python.org/downloads/
2. Install pygame:
   ```pip install pygame```
   **No need to have python knowlage just install pygame and when running the .cpp file your visualiser will work automaticaly**
