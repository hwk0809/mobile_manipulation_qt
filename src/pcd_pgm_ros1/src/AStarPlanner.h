#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <ros/ros.h>

struct Node {
    int x, y;
    double g_cost, h_cost, f_cost;
    Node* parent;

    Node(int x, int y, double g = 0, double h = 0)
        : x(x), y(y), g_cost(g), h_cost(h), f_cost(g + h), parent(nullptr) {}

    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }
};

struct NodeHash {
    std::size_t operator()(const Node& node) const {
        return std::hash<int>()(node.x) ^ std::hash<int>()(node.y);
    }
};

class AStarPlanner {
public:
    // AStarPlanner(const std::vector<std::vector<int>>& grid);
    AStarPlanner(std::vector<std::vector<int>>& grid , int inflation_radius);
    std::vector<std::pair<int, int>> findPath(const std::pair<int, int>& start,
                                              const std::pair<int, int>& goal);
    int time_max{};
    std::vector<std::vector<int>> grid_;
    std::vector<std::vector<double>> distance_map_;
    double weight_{0.5};
    double punish_k{50};

private:
    std::vector<Node> getNeighbors(const Node& node);
    double calculateHeuristic(const Node& node, const Node& goal);
    void inflateObstacles(std::vector<std::vector<int>>& grid_2d, int inflation_radius);
    // Utility functions to avoid dangling pointers
    Node* getOrCreateNode(int x, int y, double g = 0, double h = 0);
    void clearNodes();
    void computeDistanceMap();

    std::unordered_map<Node, Node*, NodeHash> node_map_;  // Node storage
};

#endif // ASTAR_PLANNER_H
