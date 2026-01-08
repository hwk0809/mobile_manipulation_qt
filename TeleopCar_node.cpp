#include "TeleopCar/TeleopCar_node.h"

namespace yhs
{
    TeleopCar::TeleopCar()
        : nh_("~")
        , cmd_velocity_(0.0)
        , cmd_steering_(0.0)
        , real_velocity_(0.0)
        , real_steering_(0.0)
        , v_scale_(1.0)
        , s_scale_(1.0)
        , gear_mod_(5)
        , running_(false)
        , current_position_(Eigen::Vector2d::Zero())
        , calculated_velocity_(Eigen::Vector2d::Zero())
        , roll(0.0)
        , pitch(0.0)
        , yaw(0.0)
    {
        // 订阅规划好的路径
        subscription_ = nh_.subscribe("/pcd2pgm/smooth_path", 10, 
            &TeleopCar::path_callback, this);
        
        // 订阅修正后的里程计数据
        odometry_subscription_ = nh_.subscribe("/pcd2pgm/modified_odometry", 10, 
            &TeleopCar::odometryCallback, this);

        // 发布可视化标记 - 修改为全局话题
        calculated_velocity_publisher_ = nh_.advertise<visualization_msgs::Marker>(
            "/calculated_velocity", 10);

        // 发布转向控制命令给车辆底盘 - 修改为全局话题
        steering_ctrl_cmd_publisher_ = nh_.advertise<yhs_can_msgs::steering_ctrl_cmd>(
            "/steering_ctrl_cmd", 1);
        // 发布速度和转向控制命令给车辆底盘 - 修改为全局话题
        ctrl_cmd_publisher_ = nh_.advertise<yhs_can_msgs::ctrl_cmd>(
            "/ctrl_cmd", 1);

        goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &TeleopCar::goalPoseCallback, this);

        // 【新增 1】：定义一个发布者，用于发布返回原点的目标点
        return_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

        // 【新增 2】：定义服务服务端，接收 Python 端发来的返回指令
        // 11111
        return_service_server_ = nh_.advertiseService("/return_task", &TeleopCar::returnTaskCallback, this);
    }

    TeleopCar::~TeleopCar()
    {
        stop();
    }

    //1.路径path的回调函数
    void TeleopCar::path_callback(const nav_msgs::Path::ConstPtr& msg)
    {
        path.clear();

        for (const auto& pose : msg->poses)
        {
            double x = pose.pose.position.x;
            double y = pose.pose.position.y;
            path.emplace_back(x, y);

            // 读取 yaw
            double qx = pose.pose.orientation.x;
            double qy = pose.pose.orientation.y;
            double qz = pose.pose.orientation.z;
            double qw = pose.pose.orientation.w;

        }
        ROS_INFO("Path size: %zu, goal_yaw: %.3f deg", path.size(), goal_yaw_ * 180.0/M_PI);
    }

    //2.车辆CurrentPosition和姿态偏航角yaw的回调函数
    void TeleopCar::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        current_position_.x() = msg->pose.pose.position.x;
        current_position_.y() = msg->pose.pose.position.y;

        // 获取四元数的姿态信息
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        
        // 将四元数转换为欧拉角（roll, pitch, yaw）
        tf2::Quaternion quat(qx, qy, qz, qw);
        tf2::Matrix3x3 mat(quat);
        mat.getRPY(roll, pitch, yaw);

        if (path.empty()) return;
        
        setGearMod(5);
        updateVelocity(current_position_);
        publishVector(current_position_[0], current_position_[1], calculated_velocity_[0], calculated_velocity_[1]);
    }

    void TeleopCar::goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        // 获取目标位置
        goal_x = msg->pose.position.x;
        goal_y = msg->pose.position.y;
        
        // 提取目标朝向（四元数转为 yaw）
        double qx = msg->pose.orientation.x;
        double qy = msg->pose.orientation.y;
        double qz = msg->pose.orientation.z;
        double qw = msg->pose.orientation.w;
        
        // 将四元数转换为 yaw
        tf2::Quaternion quat(qx, qy, qz, qw);
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw_goal;
        mat.getRPY(roll, pitch, yaw_goal);  // 提取 yaw（目标朝向）
        
        goal_yaw_ = yaw_goal;  // 保存目标的 yaw
        ROS_INFO("Received goal at position (%.2f, %.2f) with yaw %.2f", goal_x, goal_y, yaw_goal);
    }

    bool TeleopCar::returnTaskCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        ROS_INFO("Received return signal from Arm. Sending goal (0,0) to return home...");

        setCmdVelocity(-0.2);
        setCmdSteering(0.0);
        sleep(2); // 倒车2秒
        stopCar();

        // 构造 (0,0) 坐标的目标点消息
        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "map"; // 或者是您坐标系定义的 "odom"
        //22.8724
        //2.43279

        goal_msg.pose.position.x = 0.3+22.8724;
        goal_msg.pose.position.y = 1.0+2.43279;
        goal_msg.pose.position.z = 0.0;
        
        // 四元数设为单位向量，表示方向朝前
        goal_msg.pose.orientation.x = 0.0;
        goal_msg.pose.orientation.y = 0.0;
        goal_msg.pose.orientation.z = 1.0;
        goal_msg.pose.orientation.w = 0.0;

        // 发布目标点，让规划器重新计算路径回原点
        return_goal_pub_.publish(goal_msg);

        // 重置抓取状态标志位，以便下次到达后可以再次抓取
        is_grasping_active_ = false;
        reached_goal_ = false;

        res.success = true;
        res.message = "Goal (0,0) published successfully.";
        return true;
    }

    void TeleopCar::StartTask()
    {
        ROS_INFO("Received return signal from Arm. Sending goal (0,0) to return home...");

        // 构造 (0,0) 坐标的目标点消息
        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "map"; // 或者是您坐标系定义的 "odom"
        //22.8724
        //2.43279

        goal_msg.pose.position.x = 30.51;
        goal_msg.pose.position.y = 2.44;
        goal_msg.pose.position.z = 0.0;
        
        // 四元数设为单位向量，表示方向朝前
        goal_msg.pose.orientation.x = 0.0;
        goal_msg.pose.orientation.y = 0.0;
        goal_msg.pose.orientation.z = 0.0;
        goal_msg.pose.orientation.w = 1.0;

        // 发布目标点，让规划器重新计算路径回原点
        return_goal_pub_.publish(goal_msg);
    }

    void TeleopCar::switchToArmGrabMode()
    {
        // 保持之前修改好的 Trigger 调用
        // 11111
        ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("/start_grasp_task");
        std_srvs::Trigger srv;

        if (client.call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO("Mechanical arm task SUCCESS: %s", srv.response.message.c_str());
            }
            else
            {
                ROS_WARN("Mechanical arm task REJECTED: %s", srv.response.message.c_str());
                // 如果被拒绝，可以在这里将 is_grasping_active_ 重置为 false 以便重试
                is_grasping_active_ = false; 
                stopCar();
            }
        }
        else
        {
            ROS_ERROR("Failed to call service start_grasp_task.");
            is_grasping_active_ = false;
        }
    }
    //3.根据currentpos计算出速度指向，并更新全局变量
    void TeleopCar::updateVelocity(const Eigen::Vector2d position)
    {
        size_t size = path.size() - 1;
        if (path.empty())
        {
            calculated_velocity_(0) = 0;
            calculated_velocity_(1) = 0;
            std::cout << "No path found" << std::endl;
            return;
        }

        //寻找与当前位置最近的点
        double min_distance = std::numeric_limits<double>::max();
        size_t closest_index = 0;
        for (size_t i = 0; i < size; i++)
        {
            double distance = std::sqrt(
                std::pow(path[i].x() - position.x(), 2) + std::pow(path[i].y() - position.y(), 2));
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_index = i;
            }
        }

        double end_distance = std::sqrt(
                std::pow(path[size].x() - position.x(), 2) + std::pow(path[size].y() - position.y(), 2));

        // 选择前瞻点（前方10个路径点）
        size_t dot_next_index = closest_index + 10;
        if (dot_next_index >= path.size())
            dot_next_index = path.size() - 1;
        
        double goal_distance = std::sqrt(
                std::pow(goal_x - position.x(), 2) + std::pow(goal_y - position.y(), 2));
        
        // ---------- 到达目标位置后的对齐逻辑 ----------
        if (end_distance <= 0.2 && goal_distance <= 0.2)
        {
            // 进入目标对齐模式
            reached_goal_ = true;
            ROS_INFO("reached_goal_ = true;");
        }
        if(reached_goal_){

            // 计算当前 yaw 和目标 yaw 之间的误差
            double yaw_error = goal_yaw_ - yaw;
            
            // 归一化 yaw 误差到 [-pi, pi]
            while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
            
            double yaw_error_deg = yaw_error * 180.0 / M_PI;

            // 如果 yaw 误差小于某个阈值，表示已经对齐，停止
            if (std::abs(yaw_error_deg) < 10.0)  
            {
                setCmdVelocity(0.0);
                setCmdSteering(0.0);
                
                // 只有在没有进行抓取时才发起请求
                if (!is_grasping_active_) {
                    is_grasping_active_ = true; // 上锁
                    if(!grasp_or_down){
                        switchToArmGrabMode();
                        // 如果需要下次还能抓取，可以在某个条件下（如离开目标点）将 is_grasping_active_ 设为 false
                        grasp_or_down = true;
                    }
                    else{
                        // 这里可以添加放下物体的逻辑
                        // 调用放瓶子服务
                        ros::ServiceClient throw_client = nh_.serviceClient<std_srvs::Trigger>("/throw_bottle_task");
                        std_srvs::Trigger srv;
                        if (throw_client.call(srv)) {
                            ROS_INFO("Throw bottle task started.");
                        } else {
                            ROS_ERROR("Failed to call service throw_bottle_task.");
                        }

                        grasp_or_down = false;
                        stopCar();
                    }
                    
                }
                
                reached_goal_ = false;
                return;
            }

            // 如果 yaw 误差较大，则原地旋转
            // double steer_cmd = (yaw_error_deg > 0) ? 30.0 : -30.0 ;  // 根据误差决定转向
            // steer_cmd = steer_cmd / 180 *M_PI;

            setCmdVelocity(0.005);  // 停止前进
            setCmdSteering(yaw_error_deg*0.5);  // 设置转向

            ROS_INFO("Aligning yaw... yaw_error_deg=%.2f, steering=%.2f", yaw_error_deg, yaw_error_deg);
            return;
        
        }
        // ---------- 终点对齐逻辑结束 ----------


        // 计算目标方向向量
        Eigen::Vector2d theta_vector(
            path[dot_next_index].x() - position.x(),
            path[dot_next_index].y() - position.y()
        );
        
        // 计算车辆当前朝向向量
        Eigen::Matrix2d rotation_matrix;
        rotation_matrix << std::cos(yaw), -std::sin(yaw),
                           std::sin(yaw), std::cos(yaw);
        Eigen::Vector2d initial_vector(1.0, 0.0);
        Eigen::Vector2d yaw_vector = rotation_matrix * initial_vector;
        
        // 转换为3D向量以计算叉积
        Eigen::Vector3d vector_a(yaw_vector(0), yaw_vector(1), 0);
        Eigen::Vector3d vector_b(theta_vector(0), theta_vector(1), 0);
        
        // 计算夹角
        double dot_product = vector_a.dot(vector_b);
        double cos_theta = dot_product / (vector_a.norm() * vector_b.norm());
        
        // 使用 std::min 和 std::max 替代 std::clamp (C++11 兼容)
        cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
        double theta = std::acos(cos_theta);
        
        // 通过叉积判断转向方向
        double cross_product_z = vector_a.cross(vector_b).z();
        if (cross_product_z < 0) {
            theta = -theta; // 右转为负
        }
        
        // 转换为度数并设置转向角
        theta = theta * 180 / M_PI;
        setCmdSteering(theta);

        // 根据转向角调整速度
        //0.1
        setCmdVelocity(0.50 + std::max(0.0, 0.50/180*(90 - std::abs(theta))));
        //setCmdVelocity(0.3);

        // 计算速度方向向量（用于可视化）
        calculated_velocity_(0) = path[dot_next_index].x() - position.x();
        calculated_velocity_(1) = path[dot_next_index].y() - position.y();
    }

    // 可视化速度向量
    void TeleopCar::publishVector(double current_posx, double current_posy, double velocity_x, double velocity_y)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "velocity_vector";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        // 设置起点
        marker.pose.position.x = current_posx;
        marker.pose.position.y = current_posy;
        marker.pose.position.z = 0.0;

        // 计算箭头方向
        double arrow_yaw = std::atan2(velocity_y, velocity_x);
        tf2::Quaternion arrow_quat;
        arrow_quat.setRPY(0, 0, arrow_yaw);
        marker.pose.orientation = tf2::toMsg(arrow_quat);

        // 设置箭头大小
        marker.scale.x = std::sqrt(velocity_x*velocity_x + velocity_y*velocity_y) * 2.0; // 长度
        marker.scale.y = 0.1; // 宽度
        marker.scale.z = 0.1; // 高度

        // 设置颜色
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        calculated_velocity_publisher_.publish(marker);
    }

    // 启动遥控车
    void TeleopCar::start()
    {
        running_ = true;
        thread_ = std::thread(&TeleopCar::sendLoop, this);
    }

    // 停止遥控车
    void TeleopCar::stop()
    {
        running_ = false;
        if (thread_.joinable())
        {
            thread_.join();
        }
    }

    // 发送控制命令的循环
    void TeleopCar::sendLoop()
    {
        ros::Rate rate(50); // 50Hz频率
        while (running_ && ros::ok())
        {
            // 使用 FW-mini-ros1 中的 steering_ctrl_cmd 消息类型
            yhs_can_msgs::steering_ctrl_cmd cmd_msg;

            //cmd_msg.ctrl_cmd_gear = gear_mod_; // 设置档位

            // 添加一阶惯性环节
            real_velocity_ = InertialElement(8, cmd_velocity_, real_velocity_); // 更新实际速度
            real_steering_ = InertialElement(3, cmd_steering_, real_steering_); // 更新实际转向
            
            // cmd_msg.steering_ctrl_cmd_velocity = real_velocity_; // 设置速度命令
            // cmd_msg.steering_ctrl_cmd_steering = real_steering_; // 设置转向命令

            // steering_ctrl_cmd_publisher_.publish(cmd_msg);

            yhs_can_msgs::ctrl_cmd ctrl_cmd_msg;

            ctrl_cmd_msg.ctrl_cmd_gear = 6; // 设置档位
            ctrl_cmd_msg.ctrl_cmd_x_linear = real_velocity_; // 设置速度命令
            ctrl_cmd_msg.ctrl_cmd_z_angular = real_steering_; // 设置转向命令

            ctrl_cmd_publisher_.publish(ctrl_cmd_msg);
            
            rate.sleep();
        }
    }

    // 一阶惯性环节计算
    double TeleopCar::InertialElement(double tao, double GiveValue, double Value_last_time)
    {
        return (tao * Value_last_time + GiveValue) / (1 + tao);
    }

    // 设置命令速度
    void TeleopCar::setCmdVelocity(double v)
    {
        cmd_velocity_ = v; // 更新命令速度
    }

    // 设置命令转向
    void TeleopCar::setCmdSteering(double s)
    {
        cmd_steering_ = s; // 更新命令转向
    }

    // 设置档位
    void TeleopCar::setGearMod(int g)
    {
        gear_mod_ = g; // 更新档位
    }

    // 停止车辆
    void TeleopCar::stopCar()
    {
        setCmdVelocity(0.0); // 设置速度为0
        setCmdSteering(0.0); // 设置转向为0
    }

    void TeleopCar::loadPathFromFile(const std::string& filename)
    {
        // 保留此函数以保持接口兼容性
        ROS_INFO("loadPathFromFile function called with: %s", filename.c_str());
    }

    void TeleopCar::keyLoop()
    {
        // 保留此函数以保持接口兼容性
        ROS_INFO("keyLoop function called");
    }

} // namespace yhs

// 主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "TeleopCar_node"); // 初始化ROS1
    std::cout << "START" << std::endl; // 输出启动信息
    
    // 创建TeleopCar实例
    yhs::TeleopCar teleopCar;
    
    // 启动线程
    teleopCar.start();

    // 等待 1 秒，确保 Publisher 和 Subscriber 握手成功
    ros::Duration(2.0).sleep();
    teleopCar.StartTask();

    ROS_INFO("TeleopCar node started, waiting for path and odometry data...");
    

    // ROS1的事件循环
    ros::spin();
    
    // 停止车辆
    teleopCar.stop();
    
    return 0; // 返回0，正常退出
}