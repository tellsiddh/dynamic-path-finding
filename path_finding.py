import heapq
import random
import time
import matplotlib.pyplot as plt
import numpy as np

class Node:
    def __init__(self, x, y, cost, heuristic = 0, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.parent = parent

    def __lt__(self, other):
        f_self = self.cost + self.heuristic
        f_other = other.cost + other.heuristic
        if f_self == f_other:
            return self.heuristic < other.heuristic  # Tie-breaking
        return f_self < f_other

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

class DynamicObstacle:
    def __init__(self, x, y, direction):
        self.x = x
        self.y = y
        self.direction = direction

    def move(self, grid):
        new_x = self.x + self.direction[0]
        new_y = self.y + self.direction[1]

        if 0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]) and grid[new_x][new_y] == 0:
            # Swap the values
            grid[self.x][self.y], grid[new_x][new_y] = 0, 1
            self.x, self.y = new_x, new_y
        else:
            # Reverse direction if the obstacle hits a boundary or another block
            self.direction = (-self.direction[0], -self.direction[1])

def visualize_grid(grid, path=None):
    for row in range(len(grid)):
        for col in range(len(grid[0])):
            if path and (row, col) in path:
                print('P', end=' ')
            else:
                if grid[row][col] == 1:
                    print('█', end=' ')
                else:
                    print('.', end=' ')
        print()

def plot_grid(grid, path=None, obstacles=None):
    plt.imshow(grid, cmap="binary")  # Plot the grid

    if path:  # Plot the path if it exists
        path_x, path_y = zip(*path)
        plt.plot(path_y, path_x, "b.-", linewidth=2)  # Path shown in blue

    if obstacles:  # Plot the obstacles if they exist
        for obstacle in obstacles:
            plt.scatter(obstacle.y, obstacle.x, c="red", marker="o")  # Obstacles shown in red

    plt.xticks(np.arange(0.5, len(grid[0]), step=1), [])
    plt.yticks(np.arange(0.5, len(grid), step=1), [])
    plt.grid(True)
    plt.gca().invert_yaxis()  # Invert y-axis to match grid coordinates
    plt.show()

def generate_random_obstacles(grid, num_obstacles):
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    obstacles = []

    while len(obstacles) < num_obstacles:
        x = random.randint(0, len(grid) - 1)
        y = random.randint(0, len(grid[0]) - 1)

        if grid[x][y] == 0:  # If it's not an existing obstacle or terrain
            direction = random.choice(directions)
            obstacle = DynamicObstacle(x, y, direction)
            obstacles.append(obstacle)
            grid[x][y] = 1  # Block the initial position of the obstacle

    return obstacles


def a_star_with_dynamics(grid, start, goal, obstacles):
    open_list = []
    start_node = Node(start[0], start[1], 0, heuristic(start, goal))
    goal_node = Node(goal[0], goal[1], 0, heuristic(goal, goal))

    heapq.heappush(open_list, start_node)
    closed_list = set()
    max_open_list_size = 1  # Initial size of open list is 1

    moves = [
        (0, 1), (1, 0), (0, -1), (-1, 0),
        (-1, -1), (-1, 1), (1, -1), (1, 1)
    ]

    start_time = time.perf_counter()
    while open_list:
        # Move each obstacle
        for obstacle in obstacles:
            obstacle.move(grid)

        current_node = heapq.heappop(open_list)
        closed_list.add((current_node.x, current_node.y))

        if (current_node.x, current_node.y) == (goal_node.x, goal_node.y):
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            end_time = time.perf_counter()
            return path[::-1], len(closed_list), len(path), end_time - start_time, max_open_list_size, len(closed_list)

        for move in moves:
            x, y = current_node.x + move[0], current_node.y + move[1]

            if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] != 1:
                if (x, y) in closed_list:
                    continue
                next_node = Node(x, y, current_node.cost + 1, heuristic((x, y), goal), current_node)

                if next_node not in open_list:
                    heapq.heappush(open_list, next_node)

        max_open_list_size = max(max_open_list_size, len(open_list))
    
    end_time = time.perf_counter()
    return None, len(closed_list), 0, end_time - start_time, max_open_list_size, len(closed_list)


def dijkstra_dynamic(grid, start, goal, obstacles):
    open_list = []
    start_node = Node(start[0], start[1], 0, 0)
    goal_node = Node(goal[0], goal[1], 0, 0)

    heapq.heappush(open_list, start_node)
    closed_list = set()
    max_open_list_size = 1  # Initial size of open list is 1

    moves = [
        (0, 1), (1, 0), (0, -1), (-1, 0),
        (-1, -1), (-1, 1), (1, -1), (1, 1)
    ]

    start_time = time.perf_counter()
    while open_list:
        # Move each obstacle
        for obstacle in obstacles:
            obstacle.move(grid)

        current_node = heapq.heappop(open_list)
        closed_list.add((current_node.x, current_node.y))

        if (current_node.x, current_node.y) == (goal_node.x, goal_node.y):
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            end_time = time.perf_counter()
            return path[::-1], len(closed_list), len(path), end_time - start_time, max_open_list_size, len(closed_list)

        for move in moves:
            x, y = current_node.x + move[0], current_node.y + move[1]

            if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] != 1:
                if (x, y) in closed_list:
                    continue
                next_node = Node(x, y, current_node.cost + 1, 0, parent=current_node)

                if next_node not in open_list:
                    heapq.heappush(open_list, next_node)

        max_open_list_size = max(max_open_list_size, len(open_list))
    
    end_time = time.perf_counter()
    return None, len(closed_list), 0, end_time - start_time, max_open_list_size, len(closed_list)


def dfs_dynamic(grid, start, goal, obstacles):
    stack = [Node(start[0], start[1], 0)]
    closed_list = set()
    max_open_list_size = 1  # Initial size is 1

    moves = [
        (0, 1), (1, 0), (0, -1), (-1, 0),
        (-1, -1), (-1, 1), (1, -1), (1, 1)
    ]

    start_time = time.perf_counter()

    while stack:
        # Move each obstacle
        for obstacle in obstacles:
            obstacle.move(grid)

        current_node = stack.pop()
        closed_list.add((current_node.x, current_node.y))

        if (current_node.x, current_node.y) == goal:
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            end_time = time.perf_counter()
            return path[::-1], len(closed_list), len(path), end_time - start_time, max_open_list_size, len(closed_list)

        for move in moves:
            x, y = current_node.x + move[0], current_node.y + move[1]

            if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] != 1:
                if (x, y) in closed_list:
                    continue
                next_node = Node(x, y, current_node.cost + 1, parent=current_node)

                if next_node not in stack:
                    stack.append(next_node)

        max_open_list_size = max(max_open_list_size, len(stack))

    end_time = time.perf_counter()
    return None, len(closed_list), 0, end_time - start_time, max_open_list_size, len(closed_list)


from collections import deque

def bfs_dynamic(grid, start, goal, obstacles):
    queue = deque([Node(start[0], start[1], 0)])
    closed_list = set()
    max_open_list_size = 1  # Initial size is 1

    moves = [
        (0, 1), (1, 0), (0, -1), (-1, 0),
        (-1, -1), (-1, 1), (1, -1), (1, 1)
    ]

    start_time = time.perf_counter()

    while queue:
        # Move each obstacle
        for obstacle in obstacles:
            obstacle.move(grid)

        current_node = queue.popleft()
        closed_list.add((current_node.x, current_node.y))

        if (current_node.x, current_node.y) == goal:
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            end_time = time.perf_counter()
            return path[::-1], len(closed_list), len(path), end_time - start_time, max_open_list_size, len(closed_list)

        for move in moves:
            x, y = current_node.x + move[0], current_node.y + move[1]

            if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] != 1:
                if (x, y) in closed_list:
                    continue
                next_node = Node(x, y, current_node.cost + 1, parent=current_node)

                if next_node not in queue:
                    queue.append(next_node)

        max_open_list_size = max(max_open_list_size, len(queue))

    end_time = time.perf_counter()
    return None, len(closed_list), 0, end_time - start_time, max_open_list_size, len(closed_list)

def greedy_best_first_search(grid, start, goal, obstacles):
    open_list = []
    start_node = Node(start[0], start[1], 0, heuristic(start, goal))
    goal_node = Node(goal[0], goal[1], 0, 0)

    heapq.heappush(open_list, start_node)
    closed_list = set()
    max_open_list_size = 1  # Initial size of open list is 1

    moves = [
        (0, 1), (1, 0), (0, -1), (-1, 0),
        (-1, -1), (-1, 1), (1, -1), (1, 1)
    ]

    start_time = time.perf_counter()
    while open_list:
        # Move each obstacle
        for obstacle in obstacles:
            obstacle.move(grid)

        current_node = heapq.heappop(open_list)
        closed_list.add((current_node.x, current_node.y))

        if (current_node.x, current_node.y) == (goal_node.x, goal_node.y):
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            end_time = time.perf_counter()
            return path[::-1], len(closed_list), len(path), end_time - start_time, max_open_list_size, len(closed_list)

        for move in moves:
            x, y = current_node.x + move[0], current_node.y + move[1]

            if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] != 1:
                if (x, y) in closed_list:
                    continue
                next_node = Node(x, y, 0, heuristic((x, y), goal), current_node)

                if next_node not in open_list:
                    heapq.heappush(open_list, next_node)

        max_open_list_size = max(max_open_list_size, len(open_list))
    
    end_time = time.perf_counter()
    return None, len(closed_list), 0, end_time - start_time, max_open_list_size, len(closed_list)

if __name__ == "__main__":
    # Making the grid larger and more complex.
    grid = [
        [0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1],
        [0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1],
        [0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0],
        [0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0],
        [0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1],
        [0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1],
        [1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0],
        [0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0],
        [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1],
        [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0],
        [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1],
        [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0],
        [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1],
        [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0],
        [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1],
        [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0],
    ]


    start = (0, 0)
    end = (18, 0)

    # Create a dynamic obstacle and add it to the grid
    # obstacle1 = DynamicObstacle(2, 3, (1, 0))
    # obstacle2 = DynamicObstacle(5, 5, (0, 1))
    # grid[obstacle1.x][obstacle1.y] = 1  # Block the initial position of obstacle1
    # grid[obstacle2.x][obstacle2.y] = 1  # Block the initial position of obstacle2
    obstacles = generate_random_obstacles(grid, 10)  # for 5 random obstacles

    # For A* with dynamic obstacles
    path, nodes_traversed, path_length, time_taken, max_open_list_size, closed_list_size = a_star_with_dynamics(grid, start, end, obstacles)
    print(f"A* Dynamic Path: {path}")
    print(f"Nodes traversed: {nodes_traversed}")
    print(f"Path length: {path_length}")
    print(f"Time taken: {time_taken} seconds")
    print(f"Max open list size: {max_open_list_size}")
    print(f"Closed list size: {closed_list_size}")

    # For BFS with dynamic obstacles
    bfs_path_dynamic, nodes_traversed, path_length, time_taken, max_open_list_size, closed_list_size = bfs_dynamic(grid, start, end, obstacles)
    print(f"BFS Dynamic Path: {bfs_path_dynamic}")
    print(f"Nodes traversed: {nodes_traversed}")
    print(f"Path length: {path_length}")
    print(f"Time taken: {time_taken} seconds")
    print(f"Max open list size: {max_open_list_size}")
    print(f"Closed list size: {closed_list_size}")

    # For DFS with dynamic obstacles
    dfs_path_dynamic, nodes_traversed, path_length, time_taken, max_open_list_size, closed_list_size = dijkstra_dynamic(grid, start, end, obstacles)
    print(f"DFS Dynamic Path: {dfs_path_dynamic}")
    print(f"Nodes traversed: {nodes_traversed}")
    print(f"Path length: {path_length}")
    print(f"Time taken: {time_taken} seconds")
    print(f"Max open list size: {max_open_list_size}")
    print(f"Closed list size: {closed_list_size}")

    # For Dijkstra with dynamic obstacles
    dijkstra_path_dynamic, nodes_traversed, path_length, time_taken, max_open_list_size, closed_list_size = dijkstra_dynamic(grid, start, end, obstacles)
    print(f"Dijkstra Dynamic Path: {path}")
    print(f"Nodes traversed: {nodes_traversed}")
    print(f"Path length: {path_length}")
    print(f"Time taken: {time_taken} seconds")
    print(f"Max open list size: {max_open_list_size}")
    print(f"Closed list size: {closed_list_size}")

    greedy_path_dynamic, nodes_traversed, path_length, time_taken, max_open_list_size, closed_list_size = greedy_best_first_search(grid, start, end, obstacles)
    print(f"Greedy Best-First Dynamic Path: {greedy_path_dynamic}")
    print(f"Nodes traversed: {nodes_traversed}")
    print(f"Path length: {path_length}")
    print(f"Time taken: {time_taken} seconds")
    print(f"Max open list size: {max_open_list_size}")
    print(f"Closed list size: {closed_list_size}")


    if path or dijkstra_path_dynamic or dfs_path_dynamic or bfs_path_dynamic or greedy_path_dynamic:
        print("Found a path:", path)
        visualize_grid(grid, path)
        plot_grid(grid, path, obstacles)
        print("BFS Dynamic Path:", bfs_path_dynamic)
        visualize_grid(grid, bfs_path_dynamic)
        plot_grid(grid, bfs_path_dynamic, obstacles)
        print("DFS Dynamic Path:", dfs_path_dynamic)
        visualize_grid(grid, dfs_path_dynamic)
        plot_grid(grid, dfs_path_dynamic, obstacles)
        print("Dijkstra Dynamic Path:", dijkstra_path_dynamic)
        visualize_grid(grid, dijkstra_path_dynamic)
        plot_grid(grid, dijkstra_path_dynamic, obstacles)
        print("Greedy Dynamic Path:", greedy_path_dynamic)
        visualize_grid(grid, greedy_path_dynamic)
        plot_grid(grid, greedy_path_dynamic, obstacles)
    else:
        print("No path found!")
        visualize_grid(grid)
