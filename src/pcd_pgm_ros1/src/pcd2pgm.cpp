//
// Created by xiaofan on 24-11-1.
//

#include "pcd2pgm.h"
#include "AStarPlanner.h"
#include "Map2D.h"
#include "YamlReader.h"
#include <tf/transform_broadcaster.h>  // ROS1的tf
#include <tf/transform_datatypes.h>    // ROS1的tf

using namespace std;
using namespace Eigen;

using Trigger = std_srvs::Trigger;

YamlReader reader(std::string(PACKAGE_ROOT_DIR) +  "/config/config.yaml"); // this is a yaml_reader

class PointCloudToGridMap : public ros::NodeHandle {
public:

  Eigen::Vector2d x_lim_min;
  Eigen::Vector2d y_lim_min;

  double cell_resolution_{0.05}; // 栅格分辨率
  int x_cells, y_cells; // x、y 方向元胞数，即格栅地图尺寸
  int x_cells_local, y_cells_local;
  std::vector<int8_t> global_grid_map; // 格栅地图 100表示占用，0表示没被占用
  std::vector<int8_t> local_grid_map; // 局部格栅地图
  std::vector<int8_t> grid_map; // 格栅地图 是全局地图和局部地图的叠加
  MapInt8 GlobalMap; // 全局静态地图，由PCD点云生成
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  // 存储原始点云数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;  // 存储原始点云数据
  std::vector<std::pair<int, int>> path;
  Eigen::Vector3d world_t_lidar;
  Eigen::Quaterniond world_q_lidar;
  Eigen::Vector3d current_position;
  Eigen::Quaterniond current_quaternion;
  Eigen::Vector2d bias_xy;
  int queue_size; // the queue size of local map
  Eigen::Vector2d x_span,y_span;
  Eigen::Vector2d z_lim;
  Eigen::Vector3d extrinsic_T;
  Eigen::Matrix3d extrinsic_R;
  double r_expan; // 障碍物膨胀距离
  double r_expan_local; // 障碍物膨胀距离
  Eigen::Vector2d car_size_x,car_size_y;
  int time_max;
  std::vector<std::pair<int, int>> global_path;
  Vector3d O_local_map;
  bool goal_set_flag = false , flag_plan_fail = false;
  double update_k;
  double roll, pitch, yaw;
  Eigen::Vector2d angle_range; // 角度制，使用时记得转化为弧度
  int MeanK;
  double StddevMulThresh;
  std::string pcd_relevant_path;
  double smooth_k; int smooth_n;

  /************************ Constructor of PointCloudToGridMap *****************************/
  PointCloudToGridMap() : ros::NodeHandle("pcd2pgm") {
    /*************************** ROS ********************************/
    sub_all(); // subscriber
    pub_all(); // publisher
    ser_all(); // service
 
    /*********************** read para bfrom yaml **************************/
    read_yaml();
    /***************** load pcd file from pad path ***********************/
    std::string pcd_file_path = string(PACKAGE_ROOT_DIR) + pcd_relevant_path;
    cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) == -1)
    {
      ROS_ERROR("Couldn't read file\n");
      return;
    }
    cout << "load map successfully\n";

    /******************* Initialize x_lim_min and y_lim_min *******************/
    x_lim_min << -x_span[0] + current_position[0], x_span[0] + current_position[0];
    y_lim_min << -y_span[0] + current_position[1], y_span[0] + current_position[1];

    /************************ Filter point cloud *************************/
    cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::PassThrough<pcl::PointXYZ> pass; // use PassThrough filter
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_lim[0], z_lim[1]); // 我们只关心特定的高度
    pass.filter(*cloud_filtered);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor; // use StatisticalOutlierRemoval filter
    sor.setInputCloud(cloud_filtered);         // 设置输入点云
    sor.setMeanK(MeanK);                 // 设置领域点的个数，即用于计算统计数据的邻近点的数量
    sor.setStddevMulThresh(StddevMulThresh);      // 设置离群点的阈值，即标准差的倍数
    sor.filter(*cloud_filtered);   // 滤波后的结果

    /******************* Build global map ****************************/
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud_filtered, min_pt, max_pt);

    bias_xy << -min_pt[0] , -min_pt[1]; // bias_xy 一般来说是个正数
    cout << "2维地图坐标系相对3维点云坐标系偏移量为\n" << bias_xy << endl;
    publish_transform(); // tf

    x_cells = static_cast<int>((max_pt[0] - min_pt[0]) / cell_resolution_) + 1;
    y_cells = static_cast<int>((max_pt[1] - min_pt[1]) / cell_resolution_) + 1;

    GlobalMap.setMapSize({x_cells, y_cells});

    for (const auto& point : cloud_filtered->points)
    {
      int x_index = static_cast<int>((point.x - min_pt[0]) / cell_resolution_);
      int y_index = static_cast<int>((point.y - min_pt[1]) / cell_resolution_);
      if (x_index >= 0 && x_index < x_cells && y_index >= 0 && y_index < y_cells)
      {
        GlobalMap.setValue({x_index, y_index}, 100); // 标记为占用
      }
    }
    global_grid_map = GlobalMap.map_vector();
    
    /*********************** initial local map ****************************/
    x_cells_local = static_cast<int>(2 * x_span[1] / cell_resolution_) + 1;
    y_cells_local = static_cast<int>(2 * y_span[1] / cell_resolution_) + 1;
    local_grid_map.resize(x_cells_local * y_cells_local, 0);

    /******************* Create Timer with 1HZ ****************************/
    timer_ = this->createTimer(ros::Duration(1.0), &PointCloudToGridMap::timerCallback, this);
    timer2_ = this->createTimer(ros::Duration(0.1), &PointCloudToGridMap::timer2Callback, this);
 }

  /************************************下面是private部分*****************************************/
private:
  // 1Hz publish global map
  void timerCallback(const ros::TimerEvent&)
  {
    // 创建OccupancyGrid消息
    nav_msgs::OccupancyGrid grid_msg;
    grid_msg.header.frame_id = "map";
    grid_msg.header.stamp = ros::Time::now();
    grid_msg.info.resolution = cell_resolution_;
    grid_msg.info.width = x_cells;
    grid_msg.info.height = y_cells;
    grid_msg.data = grid_map;

    // 设置地图的原点
    grid_msg.info.origin.position.x = x_min;
    grid_msg.info.origin.position.y = y_min;
    grid_msg.info.origin.position.z = 0.0;
    grid_msg.info.origin.orientation.w = 1.0;

    // 发布栅格地图
    pub_map.publish(grid_msg);
    publishPointCloud(cloud_filtered);
    publishPointCloudOri(cloud);

    if(!goal_set_flag) return ; // 如果没有设定终点，说明还没有启动

    // 检查有没有遮挡，如果有重新规划全局路径
    int inflation_radius = ceil(r_expan/cell_resolution_); // 膨胀距离
    if(flag_plan_fail) { // 如果上一次规划失败，重新尝试规划
      GlobalPlanPath(inflation_radius,time_max); // 限制规划时间
      return;
    }
    if(!CheckGlobalPath()) { // 如果现有路径碰到障碍物
      GlobalPlanPath(inflation_radius,time_max); // 限制规划时间
    }
    else { // 如果没有碰到障碍物，只有效果好很多的时候才重新规划
      static int cnt = 0;
      if(cnt++ == 2) { // 每 2s 重新规划一次
        cnt = 0;
      }
      else {
        return;
      }
      if (start_.first != 0.0 && start_.second != 0.0 && goal_.first != 0.0 && goal_.second != 0.0) {
        // 将坐标转换为栅格坐标
        int start_x = static_cast<int>(start_.first / cell_resolution_);
        int start_y = static_cast<int>(start_.second / cell_resolution_);
        int goal_x = static_cast<int>(goal_.first / cell_resolution_);
        int goal_y = static_cast<int>(goal_.second / cell_resolution_);
        std::vector<std::vector<int>> grid_2d = convertTo2DGrid(grid_map, x_cells, y_cells);
        // 创建A*规划器
        AStarPlanner planner(grid_2d,inflation_radius);
        planner.time_max = time_max;
        /*************************************************************************/
        // 查找路径
        std::vector<std::pair<int, int>> temp_path = planner.findPath({start_x, start_y}, {goal_x, goal_y});
        if(! temp_path.empty() && temp_path.size() + 40 < global_path.size()) { // 效果好很多的时候才重新规划
          global_path = temp_path;
        }
        // 使用 PathPublisher 发布路径
        publishPath(global_path,"map");
        smoothPath(global_path);
      }
    }
  }

  // 10 Hz , update local map and publish local map
  void timer2Callback(const ros::TimerEvent&) {
    publish_transform(); // tf
    auto pcl_cloud_all = mergeAllClouds();
    // 清空当前范围内的grid_map
    std::fill(local_grid_map.begin(),local_grid_map.end(),0);
    // local map 原点
    O_local_map << current_position[0] - x_span[1] , current_position[1] - y_span[1],current_position[2];
    // 将点云投影到栅格地图
    std::vector<int8_t> local_grid_map_filter = local_grid_map;
    for (const auto& point : pcl_cloud_all->points)
    {
      int x_index = static_cast<int>( (point.x - O_local_map[0]) / cell_resolution_);
      int y_index = static_cast<int>( (point.y - O_local_map[1]) / cell_resolution_);
      if (x_index >= 0 && x_index < x_cells_local && y_index >= 0 && y_index < y_cells_local)
      {
        if (local_grid_map_filter[y_index * x_cells_local + x_index] < 127) {
          local_grid_map_filter[y_index * x_cells_local + x_index] += 1; // 标记为占用
        }
        local_grid_map[y_index * x_cells_local + x_index] = 100; // 标记为占用
      }
    }
    // 局部地图规划
    // LocalPlanPath();
    for (int i = 0; i<local_grid_map_filter.size()-1 ;i++) {
      if(local_grid_map_filter[i] > queue_size * update_k) {
        local_grid_map_filter[i] = 100;
      }
      else {
        local_grid_map_filter[i] = 0;
      }
    }

    // 更新临时的全局地图
    Map2D LocalMap;
    LocalMap.set_grid_map(local_grid_map_filter);
    LocalMap.set_size({x_cells_local,y_cells_local});
    LocalMap.cal_grid_map();
    LocalMap.set_current_p({x_cells_local / 2  , y_cells_local/2});
    // std::pair angle_range_pair = {angle_range[0] * M_PI /180 ,angle_range[1] * M_PI /180 };
    // 修改这一行：
    std::pair<double, double> angle_range_pair = {angle_range[0] * M_PI /180, angle_range[1] * M_PI /180};
    auto vis_index = LocalMap.vis_area( angle_range_pair, yaw);
    auto grid_2d_local = LocalMap.grid_2d;
    // 视线地图
    vector<int8_t> vis_map;
    vis_map.resize(x_cells_local * y_cells_local,0);
    // 仅对视线范围内的格栅进行更新
    grid_map = global_grid_map;
    int x_bias = static_cast<int>(O_local_map[0]/cell_resolution_);
    int y_bias = static_cast<int>(O_local_map[1]/cell_resolution_);
    for ( pair<int,int> local_index : vis_index) {
      vis_map[local_index.second * x_cells_local + local_index.first] = 50;
      int global_index = (y_bias + local_index.second) * x_cells + (x_bias + local_index.first);
      //他妈的这里不能有等号，否则会越界报错，一个小bug
      if (global_index >= 0 && global_index < x_cells * y_cells) {
        if(grid_2d_local[local_index.second][local_index.first]) {
          grid_map[global_index] = 100; // 标记为占用
        }
        else {
          grid_map[global_index] = 0; // 标记为未占用
        }
      }
    }
    publishLocalBasedMap(local_grid_map,pub_map_local);
    publishLocalBasedMap(local_grid_map_filter,pub_map_local_filter);
    publishLocalBasedMap(vis_map,pub_map_local_vision);
  }

  // 转换函数：将一维栅格地图转换为二维地图
  static std::vector<std::vector<int>> convertTo2DGrid(const std::vector<int8_t>& grid_map, int x_cells, int y_cells) {
    std::vector<std::vector<int>> grid_2d(y_cells, std::vector<int>(x_cells, 0));
    for (int y = 0; y < y_cells; ++y) {
      for (int x = 0; x < x_cells; ++x) {
        int index = y * x_cells + x;
        grid_2d[y][x] = grid_map[index] == 100 ? 1 : 0;  // 1表示占用，0表示未占用
      }
    }
    return grid_2d;
  }

  const double ANGLE_THRESHOLD = 15.0; // 角度阈值（度数）
  const double DISTANCE_THRESHOLD = 0.5; // 距离阈值（米）

  std::vector<Eigen::Vector2d> smoothPath(const std::vector<std::pair<int, int>>& path) {

    double max_error = 0.01;
    int max_depth = 10;
    double smooth_factor = 0.5;
    static int n = smooth_n; // 从path中每隔n个点取出一个控制点
    // 存储最终平滑后的路径
    std::vector<Eigen::Vector2d> smoothed_path;

    // 递归平滑核心函数
    std::function<void(const Eigen::Vector2d&, const Eigen::Vector2d&,
                       const Eigen::Vector2d&, const Eigen::Vector2d&,
                       int)> recursiveBezier = [&](const Eigen::Vector2d& p0,
                                                   const Eigen::Vector2d& p1,
                                                   const Eigen::Vector2d& p2,
                                                   const Eigen::Vector2d& p3,
                                                   int depth) {
        // 计算贝塞尔曲线中点 (t = 0.5)
        Eigen::Vector2d midPoint = 0.5 * (p3 + p0);

        // 计算贝塞尔曲线实际点 (t = 0.5)
        Eigen::Vector2d bezierPoint = 0.125 * (p0 + 3 * p1 + 3 * p2 + p3);

        // 计算误差
        double error = (midPoint - bezierPoint).norm();

        // 如果误差小于阈值或深度达到最大值，添加控制点
        if (error < max_error || depth >= max_depth) {
            smoothed_path.push_back(p0);
            return;
        }

        // 细分曲线
        Eigen::Vector2d p01 = 0.5 * (p0 + p1);
        Eigen::Vector2d p12 = 0.5 * (p1 + p2);
        Eigen::Vector2d p23 = 0.5 * (p2 + p3);

        Eigen::Vector2d p012 = 0.5 * (p01 + p12);
        Eigen::Vector2d p123 = 0.5 * (p12 + p23);

        Eigen::Vector2d p0123 = 0.5 * (p012 + p123);

        // 左段曲线
        recursiveBezier(p0, p01, p012, p0123, depth + 1);

        // 右段曲线
        recursiveBezier(p0123, p123, p23, p3, depth + 1);
    };

    // 优化控制点的分布，确保路径连续
    auto optimizeControlPoints = [&](const std::vector<std::pair<int, int>>& path) {
      std::vector<Eigen::Vector2d> optimized;
      optimized.push_back(Eigen::Vector2d(path.front().first, path.front().second)); // 添加起点

      for (size_t i = 1; i + 1 < path.size(); ++i) {
        Eigen::Vector2d prev(path[i - 1].first, path[i - 1].second);
        Eigen::Vector2d curr(path[i].first, path[i].second);
        Eigen::Vector2d next(path[i + 1].first, path[i + 1].second);

        // 根据平滑因子调整当前点
        Eigen::Vector2d optimizedPoint = curr + smooth_factor * (0.5 * (prev + next) - curr);
        optimized.push_back(optimizedPoint);
      }

      optimized.emplace_back(path.back().first, path.back().second); // 添加终点
      return optimized;
    };


    std::vector<std::pair<int, int>> new_path;
    for (size_t i = 0; i < path.size(); i += n) {
      // 确保不会超出原始路径的范围
      if (i < path.size()) {
        new_path.push_back(path[i]);
      }
    }


    // 优化路径的控制点分布
    std::vector<Eigen::Vector2d> optimizedPath = optimizeControlPoints(new_path);

    // 遍历优化后的控制点，构造贝塞尔曲线
    for (size_t i = 0; i + 3 < optimizedPath.size(); i += 3) {
        recursiveBezier(optimizedPath[i], optimizedPath[i + 1],
                        optimizedPath[i + 2], optimizedPath[i + 3], 0);
    }

    // 确保最后一个点被加入
    smoothed_path.push_back(optimizedPath.back());

    // 发布到rviz
    publishSmoothPath(smoothed_path,"map");


    return smoothed_path;
  }

  void publishPath(std::vector<std::pair<int, int>> path, const std::string& frame_id) {
    if (path.empty()) {
      std::cout << "未找到可行路径。" << std::endl;
      return;
    } else {
      std::cout << "找到路径:" << std::endl;
      for (const auto& pos : path) {
        std::cout << "(" << pos.first << ", " << pos.second << ") ";
      }
      std::cout << std::endl;
    }
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = frame_id;  // 使用"map"作为固定参考坐标系

    for (const auto& pos : path) {
      geometry_msgs::PoseStamped pose;
      pose.header = path_msg.header;

      // 将栅格坐标转换为实际位置
      pose.pose.position.x = pos.first * cell_resolution_;
      pose.pose.position.y = pos.second * cell_resolution_;
      pose.pose.position.z = 0.0;

      pose.pose.orientation.w = 1.0;  // 设置默认方向

      path_msg.poses.push_back(pose);
    }
    pub_path.publish(path_msg);
  }

  void publishLocalPath(std::vector<std::pair<int, int>> path, const std::string& frame_id) {
    static int flag = 1;
    if (path.empty()) {
      if (flag) std::cout << "未找到可行路径。" << std::endl;
      flag = 0;
      return;
    }
    flag = 1;
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = frame_id;  // 使用"map"作为固定参考坐标系

    for (const auto& pos : path) {
      geometry_msgs::PoseStamped pose;
      pose.header = path_msg.header;

      // 将栅格坐标转换为实际位置
      pose.pose.position.x = pos.first * cell_resolution_ + O_local_map[0];
      pose.pose.position.y = pos.second * cell_resolution_+ O_local_map[1];
      pose.pose.position.z = O_local_map[2];

      pose.pose.orientation.w = 1.0;  // 设置默认方向

      path_msg.poses.push_back(pose);
    }

    pub_local_path.publish(path_msg);
  }

  void publishSmoothPath(std::vector<Eigen::Vector2d> smoothed_path, const std::string& frame_id) {

    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = frame_id;  // 使用"map"作为固定参考坐标系

    for (const auto& pos : smoothed_path) {
      geometry_msgs::PoseStamped pose;
      pose.header = path_msg.header;

      // 将栅格坐标转换为实际位置
      pose.pose.position.x = pos[0] * cell_resolution_;
      pose.pose.position.y = pos[1] * cell_resolution_;
      pose.pose.position.z = 0.0;

      pose.pose.orientation.w = 1.0;  // 设置默认方向

      path_msg.poses.push_back(pose);
    }

    pub_smooth_path.publish(path_msg);
  }

  void publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
  {
    sensor_msgs::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(*cloud, point_cloud_msg);
    point_cloud_msg.header.frame_id = "map";
    point_cloud_msg.header.stamp = ros::Time::now();
    pub_cloud_.publish(point_cloud_msg);
  }

  void publishPointCloudOri(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
  {
    sensor_msgs::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(*cloud, point_cloud_msg);
    point_cloud_msg.header.frame_id = "map";
    point_cloud_msg.header.stamp = ros::Time::now();
    pub_cloud_ori.publish(point_cloud_msg);
  }

  void publishLocalBasedMap(std::vector<int8_t> grid_map, ros::Publisher &pub_map) {
    nav_msgs::OccupancyGrid grid_msg;
    grid_msg.header.frame_id = "map";
    grid_msg.header.stamp = ros::Time::now();
    grid_msg.info.resolution = cell_resolution_;
    grid_msg.info.width = x_cells_local;
    grid_msg.info.height = y_cells_local;
    grid_msg.data = grid_map;

    // 设置地图的原点
    grid_msg.info.origin.position.x = O_local_map[0];
    grid_msg.info.origin.position.y = O_local_map[1];
    grid_msg.info.origin.position.z = O_local_map[2];
    grid_msg.info.origin.orientation.w = 1.0;

    // 发布栅格地图
    pub_map.publish(grid_msg);
  }


  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    // 获取位姿信息
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;


    nav_msgs::Odometry modified_msg = *msg; // 复制原始消息

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    world_t_lidar = Eigen::Vector3d(x , y , z);
    world_q_lidar = Eigen::Quaterniond(qw,qx,qy,qz);


    Eigen::Vector3d map_t_lidar  = Eigen::Vector3d(x + bias_xy[0] , y + bias_xy[1] , z);
    Eigen::Quaterniond map_q_lidar (qw,qx,qy,qz);

    current_position = map_q_lidar *  extrinsic_T + map_t_lidar;
    current_quaternion = map_q_lidar *  extrinsic_R ;


    // current_position << x + bias_xy[0] , y + bias_xy[1] , z; // 添加偏移量
    // current_quaternion = extrinsic_R * Eigen::Quaterniond(qw,qx,qy,qz);
    // current_position = current_position + extrinsic_R * extrinsic_T;


    // 获取四元数的姿态信息
    modified_msg.pose.pose.orientation.x = current_quaternion.x();
    modified_msg.pose.pose.orientation.y = current_quaternion.y();
    modified_msg.pose.pose.orientation.z = current_quaternion.z();
    modified_msg.pose.pose.orientation.w = current_quaternion.w();

    // 计算新的位姿

    modified_msg.pose.pose.position.x = current_position[0];
    modified_msg.pose.pose.position.y = current_position[1];
    modified_msg.header.frame_id = "map";

    // 发布修改后的消息
    pub_odom.publish(modified_msg);

    start_ = {current_position[0], current_position[1]};
    // ROS_INFO("Start position set to: (%.2f, %.2f)", start_.first, start_.second);

    // 将四元数转换为欧拉角（roll, pitch, yaw）
    tf::Quaternion quat(current_quaternion.x(), current_quaternion.y(), current_quaternion.z(), current_quaternion.w());
    tf::Matrix3x3 mat(quat);

    mat.getRPY(roll, pitch, yaw);
    // 打印位姿和欧拉角信息
    //ROS_INFO("Position - x: %f, y: %f, z: %f", x, y, z);
    //ROS_INFO("Orientation (Euler) - roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);

  }

  void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // 转换点云
    pcl::fromROSMsg(*msg, *pcl_cloud);
    // 创建一个VoxelGrid滤波器对象
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    // 设置输入点云
    vg.setInputCloud(pcl_cloud);
    // 设置体素网格的大小，这里以0.01m为边长为例
    vg.setLeafSize(cell_resolution_, cell_resolution_, cell_resolution_ * 2);
    // 执行下采样
    vg.filter(*pcl_cloud);

    bool LidarFrame = false; // 点云是否在雷达坐标系下，若是则需要先变到世界系
    if (LidarFrame){
      Eigen::Matrix4f translation_matrix = Eigen::Matrix4f::Identity();  // 初始化为单位矩阵
      translation_matrix(0, 3) = world_t_lidar[0];  // 设置x方向的平移量
      translation_matrix(1, 3) = world_t_lidar[1];  // 设置y方向的平移量
      translation_matrix(2, 3) = world_t_lidar[2];  // 设置z方向的平移量
      translation_matrix.block<3, 3>(0, 0) = world_q_lidar.toRotationMatrix().cast<float>();// 设置旋转
      pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, translation_matrix);
    }

    Eigen::Matrix4f translation_matrix = Eigen::Matrix4f::Identity();  // 初始化为单位矩阵
    translation_matrix(0, 3) = bias_xy[0];  // 设置x方向的平移量
    translation_matrix(1, 3) = bias_xy[1];  // 设置y方向的平移量
    translation_matrix(2, 3) = 0;  // 设置z方向的平移量
    // translation_matrix.block<3, 3>(0, 0) = current_quaternion.toRotationMatrix().cast<float>();// 设置旋转
    // 应用平移变换
    pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, translation_matrix);
    // 准备滤波参数
    Vector2d x_lim_max (-x_span[1] + current_position[0], x_span[1] + current_position[0]);
    Vector2d y_lim_max (-y_span[1] + current_position[1], y_span[1] + current_position[1]);
    Vector2d x_lim_min (-x_span[0] + current_position[0], x_span[0] + current_position[0]);
    Vector2d y_lim_min (-y_span[0] + current_position[1], y_span[0] + current_position[1]);
    Vector2d z_lim_max ( z_lim[0] + current_position[2], z_lim[1] + current_position[2]);
    // 创建一个PassThrough滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcl_cloud);
    // 设置x轴范围
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_lim_max[0],x_lim_max[1]);
    pass.filter(*pcl_cloud);

    // 设置y轴范围
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_lim_max[0],y_lim_max[1]);
    pass.filter(*pcl_cloud);

    // 设置z轴范围
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_lim_max[0],z_lim_max[1]);
    pass.filter(*pcl_cloud);

    // 去除过近的点，使用 crop 滤波
    // 定义长方体的边界框 (min_x, min_y, min_z, max_x, max_y, max_z)
    Eigen::Vector4f min_point (x_lim_min[0] , y_lim_min[0] , z_lim_max[0] , 1.0);  // 长方体的最小坐标
    Eigen::Vector4f max_point(x_lim_min[1] , y_lim_min[1] , z_lim_max[1] , 1.0);    // 长方体的最大坐标
    // 创建 CropBox 对象
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setNegative(true);  // 去除长方体内的点，保留外部的点
    crop_box.setMin(min_point);  // 设置最小坐标
    crop_box.setMax(max_point);  // 设置最大坐标
    // 设置输入点云
    crop_box.setInputCloud(pcl_cloud);
    crop_box.filter(*pcl_cloud);

    sensor_msgs::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(*pcl_cloud, point_cloud_msg);
    point_cloud_msg.header.frame_id = "map";
    point_cloud_msg.header.stamp = ros::Time::now();
    pub_cloud_local_.publish(point_cloud_msg);

    // 添加到队列中
    addPointCloud(pcl_cloud);
  }

  // 滚动添加点云的函数
  void addPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud) {
    // 如果队列已满，删除最旧的点云
    if (pcl_clouds_buffer.size() >= queue_size) {
      pcl_clouds_buffer.pop_front();
    }
    // 添加新的点云
    pcl_clouds_buffer.push_back(new_cloud);

    // 输出当前缓冲区状态
    // std::cout << "Buffer size: " << pcl_clouds_buffer.size() << std::endl;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr mergeAllClouds() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& cloud : pcl_clouds_buffer) {
        *merged_cloud += *cloud; // 合并点云
    }
    return merged_cloud;
 }



  void publish_transform()
  {
    geometry_msgs::TransformStamped transformStamped;

    // 设置变换的参数
    transformStamped.header.stamp = ros::Time::now();
    // transformStamped.header.frame_id = "world"; // 基准框架
    // transformStamped.child_frame_id = "map"; // 新建的框架
    transformStamped.header.frame_id = "map"; // 基准框架
    transformStamped.child_frame_id = "world"; // 新建的框架
    transformStamped.transform.translation.x = -bias_xy(0); // X方向偏移量
    transformStamped.transform.translation.y = -bias_xy(1); // Y方向偏移量
    transformStamped.transform.translation.z = 0.0; // Z方向偏移量

    // 如果需要旋转，可以使用四元数设置
    tf::Quaternion q;
    q.setRPY(0, 0, 0); // 没有旋转，保持方向
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    // 发布变换
    static_tf_broadcaster_.sendTransform(transformStamped);
  }

  void startPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    start_ = {msg->pose.pose.position.x, msg->pose.pose.position.y};
    // ROS_INFO("Start position set to: (%.2f, %.2f)", start_.first, start_.second);
    // GlobalPlanPath();
  }

  void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_set_flag = true;
    goal_ = {msg->pose.position.x, msg->pose.position.y};
    ROS_INFO("Goal position set to: (%.2f, %.2f)", goal_.first, goal_.second);

    int inflation_radius = ceil(r_expan/cell_resolution_); // 膨胀距离
    GlobalPlanPath(inflation_radius,3000);
  }

  // Trigger 服务的回调函数
  bool handle_trigger(std_srvs::Trigger::Request &request, 
                      std_srvs::Trigger::Response &response) {
    // 处理请求并返回响应
    ROS_INFO("Trigger 服务被调用");
    // 使用 ostringstream 格式化字符串
    std::ostringstream oss;
    oss << "Start position set to: (" << std::fixed << std::setprecision(2)
        << start_.first << ", " << start_.second << ")";

    response.success = true;       // 假设操作成功
    response.message = oss.str();  // 将格式化后的字符串赋值给 message
    start_ = {current_position[0], current_position[1]};
    ROS_INFO("Start position set to: (%.2f, %.2f)", start_.first, start_.second);
    int inflation_radius = ceil(r_expan/cell_resolution_); // 膨胀距离
    GlobalPlanPath(inflation_radius,time_max);
    
    return true;
  }

  bool CheckGlobalPath() {
    bool flag_pass = true;
    std::vector<std::vector<int>> grid_2d = convertTo2DGrid(grid_map, x_cells, y_cells);
    int inflation_radius = ceil(r_expan/cell_resolution_);
    AStarPlanner planner(grid_2d,inflation_radius);
    grid_2d = planner.grid_;
    for (auto path_point : global_path) {
      if (grid_2d[path_point.second][path_point.first] == 1 ) {
        flag_pass = false;
        cout << "检测到障碍物，重新规划全局路径 ……" << endl;
        break;
      }
    }
    return flag_pass;
  }

  void GlobalPlanPath(int inflation_radius , int time_max) {
    if (start_.first != 0.0 && start_.second != 0.0 && goal_.first != 0.0 && goal_.second != 0.0) {
      // 将坐标转换为栅格坐标
      int start_x = static_cast<int>(start_.first / cell_resolution_);
      int start_y = static_cast<int>(start_.second / cell_resolution_);
      int goal_x = static_cast<int>(goal_.first / cell_resolution_);
      int goal_y = static_cast<int>(goal_.second / cell_resolution_);
      std::vector<std::vector<int>> grid_2d = convertTo2DGrid(grid_map, x_cells, y_cells);
      // 创建A*规划器
      AStarPlanner planner(grid_2d,inflation_radius);
      planner.time_max = time_max;
      /*************************************************************************/
      // 查找路径
      std::vector<std::pair<int, int>> temp_path = planner.findPath({start_x, start_y}, {goal_x, goal_y});
      flag_plan_fail = false;
      // if(temp_path.empty()) {
      //   temp_path.push_back({current_position[0]/cell_resolution_,current_position[1]/cell_resolution_});
      //   flag_plan_fail = true;
      // }
      // 在GlobalPlanPath函数中修改：
      if(temp_path.empty()) {
        ROS_WARN("Path planning failed: No path found from (%d, %d) to (%d, %d)", start_x, start_y, goal_x, goal_y);
        temp_path.push_back({static_cast<int>(current_position[0]/cell_resolution_), 
                       static_cast<int>(current_position[1]/cell_resolution_)});
        flag_plan_fail = true;
      }
      global_path = temp_path;
      // 使用 PathPublisher 发布路径
      publishPath(global_path,"map");
      smoothPath(global_path);
    }
  }

  void LocalPlanPath() {
    if (global_path.empty() || global_path.size() -1 < 2) // 当遇到障碍物且规划失败时，路径为当前位置点，大小为1
      return;
    int map_half_size_x = static_cast<int>(x_span[1]/cell_resolution_);
    int map_half_size_y = static_cast<int>(y_span[1]/cell_resolution_);
    int start_x = static_cast<int>(start_.first / cell_resolution_);
    int start_y = static_cast<int>(start_.second / cell_resolution_);
    int goal_x ;
    int goal_y ;
    std::vector<std::vector<int>> local_grid_2d = convertTo2DGrid(local_grid_map, x_cells_local, y_cells_local);
    int index = global_path.size()-1;
    while ( index > 0) {
      if( abs(global_path[index].first - start_x) <= map_half_size_x  && abs(global_path[index].second - start_y) <=  map_half_size_y ) {
        break; // looking back from end , when go into local map , stop decrease
      }
      index --;
    }
    // next we consider car size and push endpoint out of obstacle
    goal_x = global_path[index].first;
    goal_y = global_path[index].second;
    int inflation_radius = ceil(r_expan_local/cell_resolution_);
    AStarPlanner planner(local_grid_2d,inflation_radius);
    // planner.time_max = 50;
    // int goal_x_local = min(max(static_cast<int>( goal_x - O_local_map[0] / cell_resolution_),0),2*map_half_size_x);
    // int goal_y_local = min(max(static_cast<int>( goal_y - O_local_map[1] / cell_resolution_),0),2*map_half_size_y);
    // 在LocalPlanPath函数中修改这两行：
    int goal_x_local = std::min(std::max(static_cast<int>( goal_x - O_local_map[0] / cell_resolution_),0),2*map_half_size_x);
    int goal_y_local = std::min(std::max(static_cast<int>( goal_y - O_local_map[1] / cell_resolution_),0),2*map_half_size_y);
    /*************************************************************************/
    // 查找路径
    std::vector<std::pair<int, int>> local_path;
    local_path = planner.findPath({map_half_size_x, map_half_size_y}, {goal_x_local, goal_y_local});
    publishLocalPath(local_path,"map");
  }

  double x_min{}, y_min{};
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_local_cloud;
  std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcl_clouds_buffer;
  std::pair<double, double> start_; // the start point of Path Plan , always update to current point
  std::pair<double, double> goal_; // the end point of Path Plan , set by rviz
  ros::Timer timer_;
  ros::Timer timer2_;
  tf::TransformBroadcaster static_tf_broadcaster_;
  /*********** Publisher defination and start ************/
  ros::Publisher pub_map;
  ros::Publisher pub_map_local;
  ros::Publisher pub_map_local_filter;
  ros::Publisher pub_map_local_vision;
  ros::Publisher pub_cloud_;
  ros::Publisher pub_cloud_local_;
  ros::Publisher pub_cloud_ori;
  ros::Publisher pub_path;
  ros::Publisher pub_local_path;
  ros::Publisher pub_smooth_path;
  ros::Publisher pub_grid_map_vis;
  ros::Publisher pub_odom;
  void pub_all() {
    pub_map = this->advertise<nav_msgs::OccupancyGrid>("map", 10);
    pub_map_local = this->advertise<nav_msgs::OccupancyGrid>("local_map", 10);
    pub_map_local_filter = this->advertise<nav_msgs::OccupancyGrid>("local_map_filter", 10);
    pub_map_local_vision = this->advertise<nav_msgs::OccupancyGrid>("local_map_vision", 10);
    pub_cloud_ = this->advertise<sensor_msgs::PointCloud2>("filtered_point_cloud", 10);
    pub_cloud_local_ = this->advertise<sensor_msgs::PointCloud2>("local_point_cloud", 10);
    pub_cloud_ori = this->advertise<sensor_msgs::PointCloud2>("original_point_cloud", 10);
    pub_path = this->advertise<nav_msgs::Path>("path_Astar", 10);
    pub_local_path = this->advertise<nav_msgs::Path>("local_path_Astar", 10);
    pub_smooth_path = this->advertise<nav_msgs::Path>("smooth_path", 10);
    pub_grid_map_vis = this->advertise<sensor_msgs::PointCloud2>("grid_map_vis", 10);
    pub_odom = this->advertise<nav_msgs::Odometry>("modified_odometry", 10);
  }
  /*********** Subscription defination and start ************/
  ros::Subscriber goal_sub_; // sub goal_point from rviz
  ros::Subscriber odom_sub_; // sub odom from fastlio
  ros::Subscriber local_clould_sub_; // sub undistort point from fastlio
  void sub_all() {
    // goal_sub_ = this->subscribe<geometry_msgs::PoseStamped>(
    //     "goal_pose", 10, &PointCloudToGridMap::goalPoseCallback, this);
    goal_sub_ = this->subscribe<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 10, &PointCloudToGridMap::goalPoseCallback, this);

    odom_sub_ = this->subscribe<nav_msgs::Odometry>(
        "/LIO/odom_vehicle", 10, &PointCloudToGridMap::odom_callback, this);

    local_clould_sub_ = this->subscribe<sensor_msgs::PointCloud2>(
        "/LIO/clouds_lidar", 10, &PointCloudToGridMap::cloud_callback, this);
  }
  /*********** Service defination and Trigger ************/
  ros::ServiceServer trigger_service_;
  void ser_all() {
    trigger_service_ = this->advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
                "trigger_service",
                boost::bind(&PointCloudToGridMap::handle_trigger, this, _1, _2)
            );
  }
  /*********************** read from yaml **************************/
  void read_yaml() {
    /***************** read pcd path from yaml ***********************/
    if (reader.load()) {
      if (reader.read("path.pcd_relevant_path", pcd_relevant_path)) {
        std::cout << "pcd_relevant_path: " << pcd_relevant_path << std::endl;
      } else {
        std::cerr << "Failed to read 'pcd_relevant_path'" << std::endl;
      }
    }
    /***************** read planner para from yaml ***********************/
    if (reader.load()) {
      if (reader.read("planner.r_expan", r_expan)) {
        std::cout << "r_expan: " << r_expan << std::endl;
      } else {
        std::cerr << "Failed to read 'r_expan'" << std::endl;
      }
      if (reader.read("planner.r_expan_local", r_expan_local)) {
        std::cout << "r_expan_local: " << r_expan_local << std::endl;
      } else {
        std::cerr << "Failed to read 'r_expan_local'" << std::endl;
      }
      if (reader.readEigen("planner.car_size_x", car_size_x)) {
        std::cout << "car_size_x: " << car_size_x << std::endl;
      } else {
        std::cerr << "Failed to read 'car_size_x'" << std::endl;
      }
      if (reader.readEigen("planner.car_size_y", car_size_y)) {
        std::cout << "car_size_y: " << car_size_y << std::endl;
      } else {
        std::cerr << "Failed to read 'car_size_y'" << std::endl;
      }
      if (reader.read("planner.time_max", time_max)) {
        std::cout << "time_max: " << time_max << std::endl;
      } else {
        std::cerr << "Failed to read 'time_max'" << std::endl;
      }
    }
    /***************** read pcd file path from yaml ***********************/
    if (reader.load()) {
      if (reader.read("path.pcd_relevant_path", pcd_relevant_path)) {
        std::cout << "pcd_relevant_path: " << pcd_relevant_path << std::endl;
      } else {
        std::cerr << "Failed to read 'pcd_relevant_path'" << std::endl;
      }
    }
    /***************** read external para from yaml ***********************/
    if (reader.load()) {
      if (reader.readEigen("local_map.extrinsic_T", extrinsic_T)) {
        std::cout << "extrinsic_T: " << std::endl << extrinsic_T << std::endl;
      } else {
        std::cerr << "Failed to read 'extrinsic_T'" << std::endl;
      }
      if (reader.readEigen("local_map.extrinsic_R", extrinsic_R)) {
        std::cout << "extrinsic_R: " << std::endl << extrinsic_R << std::endl;
      } else {
        std::cerr << "Failed to read 'extrinsic_R'" << std::endl;
      }
    }
    /***************** read update_map para from yaml ***********************/
    if (reader.load()) {
      if (reader.read("update_map.update_k", update_k)) {
        std::cout << "update_k: " << update_k << std::endl;
      } else {
        std::cerr << "Failed to read 'update_k'" << std::endl;
      }
      if (reader.readEigen("update_map.angle_range", angle_range)) {
        std::cout << "angle_range: " << angle_range << std::endl;
      } else {
        std::cerr << "Failed to read 'angle_range'" << std::endl;
      }
    }
    /***************** read filter para from yaml ***********************/
    if (reader.load()) {
      if (reader.readEigen("filter.z_lim", z_lim)) {
        std::cout << "z_lim: " << std::endl << z_lim << std::endl;
      } else {
        std::cerr << "Failed to read 'z_lim'" << std::endl;
      }
      if (reader.read("filter.MeanK", MeanK)) {
        std::cout << "MeanK: " << MeanK << std::endl;
      } else {
        std::cerr << "Failed to read 'MeanK'" << std::endl;
      }
      if (reader.read("filter.StddevMulThresh", StddevMulThresh)) {
        std::cout << "StddevMulThresh: " << StddevMulThresh << std::endl;
      } else {
        std::cerr << "Failed to read 'StddevMulThresh'" << std::endl;
      }
    }
    /***************** read local_map para from yaml ***********************/
    if (reader.load()) {
      if (reader.read("local_map.queue_size", queue_size)) {
        std::cout << "queue_size: " << queue_size << std::endl;
      } else {
        std::cerr << "Failed to read 'queue_size'" << std::endl;
      }
      if (reader.readEigen("local_map.x_span", x_span)) {
        std::cout << "x_span: " << std::endl << x_span << std::endl;
      } else {
        std::cerr << "Failed to read 'x_span'" << std::endl;
      }
      if (reader.readEigen("local_map.y_span", y_span)) {
        std::cout << "y_span: " << std::endl << y_span << std::endl;
      } else {
        std::cerr << "Failed to read 'y_span'" << std::endl;
      }
    }
    /***************** read path_smooth para from yaml ***********************/
    if (reader.load()) {
      if (reader.read("path_smooth.smooth_k", smooth_k)) {
        std::cout << "smooth_k: " << smooth_k << std::endl;
      } else {
        std::cerr << "Failed to read 'smooth_k'" << std::endl;
      }
      if (reader.read("path_smooth.smooth_n", smooth_n)) {
        std::cout << "smooth_n: " << std::endl << smooth_n << std::endl;
      } else {
        std::cerr << "Failed to read 'smooth_n'" << std::endl;
      }
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcd2pgm");
  PointCloudToGridMap node;  // 直接创建对象，不使用shared_ptr
  ros::spin();
  return 0;
}
