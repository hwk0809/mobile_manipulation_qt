//
// Created by xiaofan on 24-11-27.
//

#ifndef INC_MAP2D_H
#define INC_MAP2D_H
#include <cstdint>
#include <vector>
#include <cmath>
#include <utility>
#include <stdexcept>
#include <iostream>
#include <queue>

class Map2D {
public:
    Map2D(); // 默认构造函数
    Map2D(std::vector<int8_t> grid_map, std::pair<int,int> size); // 构造函数
    std::vector<int8_t> grid_map; // 栅格地图 一维数组表示 100表示占用，0表示没被占用
    std::vector<std::vector<int>> grid_2d; // 2维栅格地图
    std::pair<int,int> size; // 栅格地图大小
    void cal_grid_map(); // 由1维栅格地图计算得到2维地图
    static std::vector<std::vector<int>> cal_grid_map(std::vector<int8_t> grid_map , std::pair<int,int> size); // 由1维栅格地图计算得到2维地图
    void set_grid_map(std::vector<int8_t> grid_map); // 设置格栅地图
    void set_size(std::pair<int,int> size); // 设置地图尺寸
    void set_origin(std::pair<int,int> origin_); // 设置栅格地图原点
    void set_current_p(std::pair<int,int> current_p_); // 设置当前位置
    std::vector<std::pair<int, int>> vis_area(); // 从当前位置向四周环视，可看到的区域
    std::vector<std::pair<int, int>> vis_area( std::pair<double , double> angle_range , double yaw); // 从当前位置向四周环视，可看到的区域，视野限制为前方的一个角度

private:
    std::pair<int,int> origin_ ; // 栅格地图原点（相对于全局坐标系）
    std::pair<int,int> current_p_; // 相对于栅格地图原点的当前位置
    std::pair<double , double> angle_range_;
    double yaw_{}; // 小车当前方向
    bool angle_filter_en_{};

};


class MapInt8 {
private:
    std::vector<int8_t> map_vector_;
    std::pair<int,int> size_pair_;
    void check_init_(); // 检查是否初始化

public:
    MapInt8(); // 默认构造函数
    void setMapVec(const std::vector<int8_t>& map_vector, std::pair<int,int> size_pair); // 传入一维Map向量和Map的大小并初始化
    void setMapSize(std::pair<int,int> size_pair); // 仅传入Map的大小并初始化为全零
    void setValue(int index, int8_t value); // 设置一维map中某个位置的值
    void setValue(std::pair<int,int> pos_pair, int8_t value); // 设置二维map中某个位置的值
    std::vector<int8_t> map_vector(); // 返回一维map
    std::vector<std::vector<int>> map_2d(); // 返回二维map
    std::pair<int,int> index2pos(int index); // 从索引到二维坐标
    int pos2index(std::pair<int,int> pos_pair); // 从二维坐标到索引
    void printMap2D(); // 调试用
};

// 由占用格栅初始化距离地图
class MapDistance {
private:
    MapInt8 map_int8_;
    std::vector<double> map_vector_;
    std::pair<int,int> size_pair_;
    void check_init_(); // 检查是否初始化
    void computeDistanceMap_(); // 计算距离地图

public:
    MapDistance(); // 默认构造函数
    void setMapOcc(MapInt8 map_int8_); // 传入占用地图并初始化
    std::vector<std::vector<double>> distance_map(); // 返回二维距离map
};

#endif //INC_MAP2D_H
