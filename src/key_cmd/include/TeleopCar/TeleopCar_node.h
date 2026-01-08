#ifndef TELEOPCAR_H
#define TELEOPCAR_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// 使用 FW-mini-ros1 中的消息类型
#include <yhs_can_msgs/steering_ctrl_cmd.h>
#include <yhs_can_msgs/steering_ctrl_fb.h>
#include <yhs_can_msgs/ctrl_cmd.h>
#include <yhs_can_msgs/ctrl_fb.h>

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>

#include "std_srvs/Trigger.h"  // 可以自定义服务类型

#define KEYCODE_R 0x43  // 右箭头
#define KEYCODE_L 0x44  // 左箭头
#define KEYCODE_U 0x41  // 上箭头
#define KEYCODE_D 0x42  // 下箭头
#define KEYCODE_Q 0x71  // Q键
#define KEYCODE_S 0x73  // S键
#define KEYCODE_C 0x63  // C键

namespace yhs
{
    class TeleopCar
    {
    public:
        TeleopCar();
        ~TeleopCar();

        void start();
        void stop();
        void keyLoop();
        void loadPathFromFile(const std::string& filename);

        /*
         *新增部分：先从本地数据读取当前位置和地图中全部路径
         *by 张天帅
         */
        std::vector<double> current_position;
        std::vector<Eigen::Vector2d> path;

        //终点对齐
        double goal_yaw_;  // 目标的 yaw 值
        double goal_x;  // 目标的 yaw 值
        double goal_y;  // 目标的 yaw 值
        bool reached_goal_ = false;  // 标志：是否到达目标

        void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void publishVector(double current_posx, double current_posy, double velocity_x, double velocity_y);
        double roll, pitch, yaw;

        // 成员函数
        bool returnTaskCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        void StartTask();
        /*
         *新增结束by张天帅
         */

    private:
        ros::NodeHandle nh_;
        double cmd_velocity_, cmd_steering_, real_velocity_, real_steering_, v_scale_, s_scale_;
        uint8_t gear_mod_;
        
        // ROS1 subscribers and publishers
        ros::Subscriber subscription_;
        ros::Subscriber odometry_subscription_;

        ros::Subscriber goal_sub_; // 订阅目标位置

        ros::Publisher steering_ctrl_cmd_publisher_;
        ros::Publisher ctrl_cmd_publisher_;
        ros::Publisher calculated_velocity_publisher_;

        // 成员变量
        ros::ServiceServer return_service_server_;
        ros::Publisher return_goal_pub_;

        // 路径回调函数
        void path_callback(const nav_msgs::Path::ConstPtr& msg);

        // 成员变量
        Eigen::Vector2d current_position_;
        Eigen::Vector2d calculated_velocity_;
        bool is_grasping_active_ = false; // 初始设为 false

        bool grasp_or_down = false; // 初始设为 false


        std::atomic<bool> running_;
        std::thread thread_;


        // 控制相关函数
        void sendLoop();
        void switchToArmGrabMode();
        void updateVelocity(const Eigen::Vector2d position);
        void setCmdVelocity(double v);
        void setCmdSteering(double s);
        void setGearMod(int g);
        void stopCar();
        
        // 一阶惯性环节
        double InertialElement(double tao, double GiveValue, double Value_last_time);
    };
}

#endif //TELEOPCAR_H