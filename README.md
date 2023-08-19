# Dynamic Pathfinding Algorithms

A collection of pathfinding algorithms including A*, BFS, DFS, Dijkstra, and Greedy Best-First Search, with support for dynamic obstacles.

<!-- ![Pathfinding Visualization](./pathfinding_visualization.png) Replace with actual image link -->

## Table of Contents
1. [Introduction](#introduction)
2. [Features](#features)
3. [Installation](#installation)
4. [Usage](#usage)

## Introduction
This project is a simulation of pathfinding algorithms on a grid with static and dynamic obstacles. The algorithms implemented include A*, BFS, DFS, Dijkstra, and Greedy Best-First Search. The dynamic obstacles can move within the grid, providing a real-time challenge for the algorithms to adapt and find the path.

## Features
- **A* Algorithm**: Efficient pathfinding algorithm that uses a heuristic.
- **Breadth-First Search (BFS)**: Finds the shortest path without considering the heuristic.
- **Depth-First Search (DFS)**: Explores as far as possible along each branch before backtracking.
- **Dijkstra's Algorithm**: Similar to A*, but considers all possible paths.
- **Greedy Best-First Search**: Utilizes heuristics to make decisions.
- **Dynamic Obstacles**: Obstacles that can move within the grid, creating a dynamic environment.
- **Visualization Tools**: Visualize the grid, paths, and dynamic obstacles using matplotlib.

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/tellsiddh/dynamic-path-planning.git

2. Install required dependencies:
    ```bash
    pip install matplotlib numpy

## Usage

Modify the grid, start, and end variables as desired.

Run the main script:

    ```bash
    Copy code
    python pathfinding.py

Visualizations and statistics about the paths found will be displayed in the terminal and graphical plots.