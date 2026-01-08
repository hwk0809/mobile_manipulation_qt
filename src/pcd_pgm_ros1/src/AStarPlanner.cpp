#include "AStarPlanner.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
// #include <geometry_msgs/msg/detail/quaternion__struct.hpp>

// #include <rclcpp/exceptions/exceptions.hpp>


// AStarPlanner::AStarPlanner(const std::vector<std::vector<int>>& grid) : grid_(grid) {}
AStarPlanner::AStarPlanner( std::vector<std::vector<int>>& grid , int inflation_radius) {
    AStarPlanner::inflateObstacles(grid , inflation_radius);
    grid_ = grid;
    int rows = grid_.size();
    int cols = grid_[0].size();
    distance_map_.resize(rows, std::vector<double>(cols, std::numeric_limits<double>::max()));
    computeDistanceMap();
    time_max = 5000;
}

void AStarPlanner::inflateObstacles(std::vector<std::vector<int>>& grid_2d, int inflation_radius) {
    int y_cells = grid_2d.size();
    int x_cells = grid_2d[0].size();
    std::vector<std::vector<int>> inflated_grid = grid_2d;

    for (int y = 0; y < y_cells; ++y) {
        for (int x = 0; x < x_cells; ++x) {
            if (grid_2d[y][x] == 1) {  // 如果是障碍物
                for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
                    for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
                        int ny = y + dy;
                        int nx = x + dx;
                        if (ny >= 0 && ny < y_cells && nx >= 0 && nx < x_cells) {
                            inflated_grid[ny][nx] = 1;  // 扩展为障碍物
                        }
                    }
                }
            }
        }
    }
    grid_2d = inflated_grid;  // 更新原栅格地图
}

// 广度优先搜索函数，用于找到离障碍物最近的非占据点
std::pair<int, int> bfs(int startX, int startY, std::vector<std::vector<int>> grid_) {
    int rows = grid_.size();
    int cols = grid_[0].size();
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::queue<std::pair<int, int>> q;
    std::vector<int> dx = {0, 0, -1, 1};
    std::vector<int> dy = {1, -1, 0, 0};

    q.push({startX, startY});
    visited[startY][startX] = true;

    while (!q.empty()) {
        auto current = q.front();
        q.pop();
        int x = current.first;
        int y = current.second;

        // 检查当前点是否为非占据点
        if (grid_[y][x] == 0) {
            return {x, y};
        }

        // 遍历四个方向
        for (int i = 0; i < 4; ++i) {
            int newX = x + dx[i];
            int newY = y + dy[i];

            // 检查新位置是否合法且未访问过
            if (newX >= 0 && newX < cols && newY >= 0 && newY < rows && !visited[newY][newX]) {
                q.push({newX, newY});
                visited[newY][newX] = true;
            }
        }
    }
    // 如果没有找到非占据点，返回无效坐标
    return {};
}

// 搜索初始目标点一定半径内的点
std::pair<int, int> searchRadius(int startX, int startY, std::vector<std::vector<int>> grid_, int radius) {
    int rows = grid_.size();
    int cols = grid_[0].size();
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            int newX = startX + dx;
            int newY = startY + dy;
            if (newX >= 0 && newX < cols && newY >= 0 && newY < rows && grid_[newY][newX] == 0) {
                return {newX, newY};
            }
        }
    }
    return {};
}

std::vector<std::pair<int, int>> AStarPlanner::findPath(const std::pair<int, int>& start,
                                                        const std::pair<int, int>& goal) {
    std::pair<int, int> newGoal = goal;
    if (grid_[start.second][start.first] == 1 ) {
        clearNodes();
        std::cout << "初始位置在障碍物内，无法规划" << std::endl;
        return {};
    }
    else if(grid_[goal.second][goal.first] == 1) {
        std::cout << "目标位置在障碍物内，无法规划" << std::endl;
        // 如果目标点在障碍物内，导航到障碍物附近
        newGoal = bfs(goal.first, goal.second,grid_);
        if (newGoal.first != -1 && newGoal.second != -1) {
            std::cout << "找到新的目标点: (" << newGoal.first << ", " << newGoal.second << ")" << std::endl;
        } else {
            std::cout << "BFS未找到合适的新目标点，尝试搜索半径内的点..." << std::endl;
            int searchRadiusValue = 5; // 设置搜索半径
            newGoal = searchRadius(goal.first, goal.second, grid_, searchRadiusValue);
            if (newGoal.first != -1 && newGoal.second != -1) {
                std::cout << "找到半径内的新目标点: (" << newGoal.first << ", " << newGoal.second << ")" << std::endl;
            } else {
                std::cout << "半径内也未找到合适的新目标点" << std::endl;
                return {};
            }
            // std::cout << "未找到合适的新目标点" << std::endl;
            // return {};
        }
        // return {};
    }



    Node* startNode = getOrCreateNode(start.first, start.second);
    Node* goalNode = getOrCreateNode(newGoal.first, newGoal.second);

    auto compare = [](Node* a, Node* b) { return a->f_cost > b->f_cost; };
    std::priority_queue<Node*, std::vector<Node*>, decltype(compare)> openSet(compare);
    openSet.push(startNode);

    std::unordered_map<Node, bool, NodeHash> closedSet;

    auto start_time = std::chrono::system_clock::now();  // 记录开始时间
    std::chrono::milliseconds timeout(time_max);  // 设置超时时间

    while (!openSet.empty()) {
        // overtime will quit
        auto current_time = std::chrono::system_clock::now();  // 获取当前时间
        std::chrono::duration<double> elapsed = current_time - start_time;  // 计算已过时间
        if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > timeout.count()) {
            std::cout << "Error! time out when planning , please set larger time_max or try again" << std::endl;
            break;  // 如果超过超时时间，则退出循环
        }
        Node* current = openSet.top();
        openSet.pop();
        if (*current == *goalNode) {
            std::vector<std::pair<int, int>> path;
            while (current) {
                path.emplace_back(current->x, current->y);
                current = current->parent;
            }
            clearNodes();
            std::reverse(path.begin(), path.end());
            std::cout << "Path Find" << std::endl;
            return path;
        }
        closedSet[*current] = true;
        for (const auto& neighbor : getNeighbors(*current)) {
            if (closedSet.find(neighbor) != closedSet.end() || grid_[neighbor.y][neighbor.x] == 1)
                continue;

            Node* neighborNode = getOrCreateNode(neighbor.x, neighbor.y);
            double tentative_g_cost = current->g_cost + 1.0;

            if (tentative_g_cost < neighborNode->g_cost || neighborNode->g_cost == 0) {
                neighborNode->parent = current;
                neighborNode->g_cost = tentative_g_cost;
                neighborNode->h_cost = calculateHeuristic(*neighborNode, *goalNode);
                neighborNode->f_cost = neighborNode->g_cost + neighborNode->h_cost;
                openSet.push(neighborNode);
            }
        }
    }

    clearNodes();
    return {};  // Return empty path if goal is not reachable
}

std::vector<Node> AStarPlanner::getNeighbors(const Node& node) {
    std::vector<Node> neighbors;
    int dx[4] = {-1, 1, 0, 0};
    int dy[4] = {0, 0, -1, 1};

    for (int i = 0; i < 4; ++i) {
        int nx = node.x + dx[i];
        int ny = node.y + dy[i];
        if (nx >= 0 && ny >= 0 && ny < grid_.size() && nx < grid_[0].size()) {
            neighbors.emplace_back(nx, ny);
        }
    }
    return neighbors;
}

double AStarPlanner::calculateHeuristic(const Node& node, const Node& goal) {
    double h = std::abs(node.x - goal.x) + std::abs(node.y - goal.y);
    if (node.x < 0 || node.y < 0 || node.y >= grid_.size() || node.x >= grid_[0].size()) {
        throw std::out_of_range("Node out of bounds.");
    }
    double p = punish_k / (distance_map_[node.y][node.x] + 1e-5);  // 距离越小，惩罚越大（避免除零）
    return h + weight_ * p;
}

Node* AStarPlanner::getOrCreateNode(int x, int y, double g, double h) {
    Node tempNode(x, y);
    auto it = node_map_.find(tempNode);
    if (it == node_map_.end()) {
        Node* newNode = new Node(x, y, g, h);
        node_map_[tempNode] = newNode;
        return newNode;
    }
    return it->second;
}

void AStarPlanner::clearNodes() {
    for (auto& pair : node_map_) {
        delete pair.second;
    }
    node_map_.clear();
}


void AStarPlanner::computeDistanceMap() {
    int rows = grid_.size();
    int cols = grid_[0].size();
    // std::vector<std::vector<double>> distance_map(rows, std::vector<double>(cols, std::numeric_limits<double>::max()));
    // distance_map_.resize(rows, std::vector<double>(cols, std::numeric_limits<double>::max()));
    // 初始化障碍物位置
    std::queue<std::pair<int, int>> q;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (grid_[i][j] == 1) {  // 1 表示障碍物
                distance_map_[i][j] = 0;
                q.emplace(i, j);
            }
        }
    }

    // 广度优先搜索计算每个点到最近障碍物的距离
    std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    while (!q.empty()) {
        std::pair<int, int> current = q.front();
        q.pop();
        int x = current.first;
        int y = current.second;
        for (size_t i = 0; i < directions.size(); ++i) {
            int dx = directions[i].first;
            int dy = directions[i].second;
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < rows && ny >= 0 && ny < cols) {
                double new_distance = distance_map_[x][y] + 1.0;  // 曼哈顿距离
                if (new_distance < distance_map_[nx][ny]) {
                    distance_map_[nx][ny] = new_distance;
                    q.emplace(nx, ny);
                }
            }
        }
    }
}
