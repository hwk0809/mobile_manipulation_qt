//
// Created by xiaofan on 24-11-27.
//

#include "Map2D.h"

#include <kdl/utilities/utility.h>

// 默认构造函数
Map2D::Map2D() = default;

// 构造函数
Map2D::Map2D(std::vector<int8_t> grid_map, std::pair<int, int> size)
    : grid_map(grid_map), size(size), origin_{0, 0}, current_p_{0, 0}, angle_filter_en_(false)
{
    cal_grid_map();
}

// 由1维栅格地图计算得到2维地图
void Map2D::cal_grid_map()
{
    grid_2d = Map2D::cal_grid_map(grid_map, size);
}

void Map2D::set_grid_map(std::vector<int8_t> grid_map)
{
    Map2D::grid_map = grid_map;
}

void Map2D::set_size(std::pair<int, int> size)
{
    Map2D::size = size;
}

// 设置栅格地图原点
void Map2D::set_origin(std::pair<int, int> origin)
{
    origin_ = origin;
}

// 设置当前位置
void Map2D::set_current_p(std::pair<int, int> current_p)
{
    current_p_ = current_p;
}

// 从当前位置向四周环视，可看到的区域，视野限制为前方的一个角度

std::vector<std::pair<int, int>> Map2D::vis_area(std::pair<double, double> angle_range, double yaw)
{
    angle_range_ = angle_range;
    yaw_ = yaw;
    angle_filter_en_ = true;
    return Map2D::vis_area();
}

// 从当前位置向周围环视，可看到的区域

std::vector<std::pair<int, int>> Map2D::vis_area()
{
    std::vector<std::pair<int, int>> explored; // 存储被探索的栅格坐标
    int start_x = current_p_.first;
    int start_y = current_p_.second;

    // 计算外层一圈栅格的坐标
    int x_cells = size.first;
    int y_cells = size.second;

    // 用来避免重复探索的栅格
    std::vector<std::vector<bool>> visited(y_cells, std::vector<bool>(x_cells, false));

    // 存储最外层栅格的坐标
    std::vector<std::pair<int, int>> outer_cells;

    // 上边界
    for (int x = 0; x < x_cells; ++x)
    {
        outer_cells.push_back({x, 0});
    }

    // 下边界
    for (int x = 0; x < x_cells; ++x)
    {
        outer_cells.push_back({x, y_cells - 1});
    }

    // 左边界
    for (int y = 1; y < y_cells - 1; ++y)
    {
        outer_cells.push_back({0, y});
    }

    // 右边界
    for (int y = 1; y < y_cells - 1; ++y)
    {
        outer_cells.push_back({x_cells - 1, y});
    }

    // 遍历每个最外层栅格，计算相对当前位置的角度并发射射线
    for (const auto &target : outer_cells)
    {
        int target_x = target.first;
        int target_y = target.second;

        // 计算当前位置到目标栅格的相对角度（以弧度为单位）
        double dx = target_x - start_x;
        double dy = target_y - start_y;
        double angle = std::atan2(dy, dx); // 使用 atan2 计算角度
        double angle_to_yaw = angle - yaw_;
        if (angle_to_yaw > M_PI) angle_to_yaw -= 2 * M_PI;
        if (angle_to_yaw < -M_PI) angle_to_yaw += 2 * M_PI;

        if (angle_filter_en_ && (angle_to_yaw < angle_range_.first || angle_to_yaw > angle_range_.second))
        {
            continue; // 如果在一定角度之外则不更新
        }

        // 从当前位置沿这个角度发射射线
        int x = start_x;
        int y = start_y;
        double x_d = x;
        double y_d = y;
        while (x >= 0 && x < x_cells && y >= 0 && y < y_cells)
        {
            // 如果当前格栅被占用，停止
            if (grid_2d[y][x] == 1)
            {
                explored.push_back({x, y});
                break;
            }

            // 如果当前格栅已经被探索过了，则跳过该格栅
            if (!visited[y][x])
            {
                visited[y][x] = true;       // 标记为已探索
                explored.push_back({x, y}); // 加入被探索的栅格
            }

            // 逐步向目标方向推进
            double step_x = cos(angle);
            double step_y = sin(angle);
            x_d += step_x;
            y_d += step_y;
            x = std::round(x_d);
            y = std::round(y_d);

            // 如果到达地图边界，停止
            if (x < 0 || x >= x_cells || y < 0 || y >= y_cells)
            {
                break;
            }
        }
    }

    return explored;
}

// 由1维栅格地图计算得到2维地图（静态成员函数）
std::vector<std::vector<int>> Map2D::cal_grid_map(std::vector<int8_t> grid_map, std::pair<int, int> size)
{
    int x_cells = size.first;
    int y_cells = size.second;
    std::vector<std::vector<int>> grid_2d(y_cells, std::vector<int>(x_cells, 0));
    for (int y = 0; y < y_cells; ++y)
    {
        for (int x = 0; x < x_cells; ++x)
        {
            int index = y * x_cells + x;
            grid_2d[y][x] = grid_map[index] == 100 ? 1 : 0; // 1表示占用，0表示未占用
        }
    }
    return grid_2d;
}

/**************************** MapInt类实现 *******************************/

MapInt8::MapInt8() : size_pair_(0, 0) {}

// 初始化函数，传入一维地图数据和尺寸
void MapInt8::setMapVec(const std::vector<int8_t> &map_vector, std::pair<int, int> size_pair)
{
    if (map_vector.size() != size_pair.first * size_pair.second)
    {
        throw std::invalid_argument("The size of map_vector does not match the specified size_pair.");
    }
    map_vector_ = map_vector;
    size_pair_ = size_pair;
}

// 初始化参数，通过尺寸初始化
void MapInt8::setMapSize(std::pair<int, int> size_pair)
{
    size_pair_ = size_pair;
    map_vector_.resize(size_pair_.first * size_pair_.second, 0);
}

// 设置值
void MapInt8::setValue(int index, int8_t value)
{
    check_init_();
    map_vector_[index] = value;
}

// 设置二维map中某个位置的值
void MapInt8::setValue(std::pair<int, int> pos_pair, int8_t value)
{
    int index = pos2index(pos_pair);
    setValue(index, value);
}

// 返回一维地图向量
std::vector<int8_t> MapInt8::map_vector()
{
    check_init_();
    return map_vector_;
}

// 返回二维地图
std::vector<std::vector<int>> MapInt8::map_2d()
{
    check_init_();
    std::vector<std::vector<int>> map_2d;
    map_2d.resize(size_pair_.second, std::vector<int>(size_pair_.first, 0));
    for (int i = 0; i < size_pair_.second; ++i)
    {
        for (int j = 0; j < size_pair_.first; ++j)
        {
            map_2d[i][j] = map_vector_[i * size_pair_.first + j];
        }
    }
    return map_2d;
}

// 从一维索引转换为二维坐标
std::pair<int, int> MapInt8::index2pos(int index)
{
    check_init_();
    if (index < 0 || index >= size_pair_.first * size_pair_.second)
    {
        throw std::out_of_range("Index out of bounds.");
    }
    int y = index / size_pair_.first; // 行
    int x = index % size_pair_.first; // 列
    return {x, y};
}

// 从二维坐标转换为一维索引
int MapInt8::pos2index(std::pair<int, int> pos_pair)
{
    check_init_();
    int x = pos_pair.first;
    int y = pos_pair.second;

    if (x < 0 || x >= size_pair_.first || y < 0 || y >= size_pair_.second)
    {
        throw std::out_of_range("Position out of bounds.");
    }
    return y * size_pair_.first + x; // 根据二维坐标计算一维索引
}

// 打印二维地图，方便调试
void MapInt8::printMap2D()
{
    check_init_();
    for (int i = 0; i < size_pair_.second; ++i)
    {
        for (int j = 0; j < size_pair_.first; ++j)
        {
            int index = i * size_pair_.first + j;
            std::cout << map_vector_[index] << " "; // 打印每个格子的值
        }
        std::cout << std::endl;
    }
}

// 检查是否初始化
void MapInt8::check_init_()
{
    if (size_pair_.first == 0 || size_pair_.second == 0 || map_vector_.size() == 0)
    {
        throw std::invalid_argument("The map has not been initialized.");
    }
}

/**************************** MapDistance类实现 *******************************/

MapDistance::MapDistance() : size_pair_(0, 0) {}
