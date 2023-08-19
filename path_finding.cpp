#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <utility>
#include <algorithm>

using namespace std;

struct Node {
    int x, y;
    int cost;
    int heuristic;
    Node* parent;

    Node(int _x, int _y, int _cost, int _heuristic, Node* _parent = nullptr)
        : x(_x), y(_y), cost(_cost), heuristic(_heuristic), parent(_parent) {}

    int f() const {
        return cost + heuristic;
    }

    bool operator<(const Node& other) const {
        return f() > other.f();
    }
};

int heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

vector<pair<int, int>> a_star(const vector<vector<int>>& grid, pair<int, int> start, pair<int, int> goal) {
    priority_queue<Node> open_list;
    set<pair<int, int>> closed_list;

    open_list.emplace(start.first, start.second, 0, heuristic(start.first, start.second, goal.first, goal.second));

    while (!open_list.empty()) {
        Node current_node = open_list.top();
        open_list.pop();
        closed_list.insert({current_node.x, current_node.y});

        if (current_node.x == goal.first && current_node.y == goal.second) {
            vector<pair<int, int>> path;
            while (current_node.parent) {
                path.push_back({current_node.x, current_node.y});
                current_node = *current_node.parent;
            }
            path.push_back({current_node.x, current_node.y});
            reverse(path.begin(), path.end());
            return path;
        }

        vector<pair<int, int>> neighbors = {
            {current_node.x - 1, current_node.y},
            {current_node.x + 1, current_node.y},
            {current_node.x, current_node.y - 1},
            {current_node.x, current_node.y + 1}
        };

        for (const auto& [x, y] : neighbors) {
            if (x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size() && grid[x][y] == 0) {
                if (closed_list.find({x, y}) != closed_list.end()) continue;
                int new_cost = current_node.cost + 1;
                open_list.emplace(x, y, new_cost, heuristic(x, y, goal.first, goal.second), new Node(current_node));
            }
        }
    }

    return {};  // No path found
}

int main() {
    vector<vector<int>> grid = {
        {0, 1, 0, 0, 0, 0},
        {0, 1, 0, 1, 1, 0},
        {0, 0, 0, 1, 0, 0},
        {0, 1, 1, 1, 1, 0},
        {0, 0, 0, 0, 0, 0}
    };

    pair<int, int> start = {0, 0};
    pair<int, int> end = {4, 5};

    vector<pair<int, int>> path = a_star(grid, start, end);
    if (!path.empty()) {
        cout << "Found a path:" << endl;
        for (const auto& [x, y] : path) {
            cout << "(" << x << ", " << y << ")" << endl;
        }
    } else {
        cout << "No path found!" << endl;
    }

    return 0;
}
