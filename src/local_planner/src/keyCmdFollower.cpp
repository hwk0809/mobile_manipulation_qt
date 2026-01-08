#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <yhs_can_msgs/ctrl_cmd.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int8.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

class KeyCmdFollower
{
public:
    KeyCmdFollower() : nh_("~"), path_received_(false), odom_received_(false), reached_goal_(false), goal_yaw_(0.0), current_yaw_(0.0), 
                      real_velocity_(0.0), real_steering_(0.0),
                      turnAroundTriggered_(false), turnAroundInProgress_(false), turnAroundCompleted_(false),
                      turnAroundTargetYaw_(0.0), turnAroundStartYaw_(0.0)
    {
        // Parameters
        std::string odom_topic;
        std::string path_topic;
        nh_.param<std::string>("odom_topic", odom_topic, "/odom");
        nh_.param<std::string>("path_topic", path_topic, "/local_path");
        nh_.param<double>("lookahead_dist_index", lookahead_index_offset_, 10.0);
        nh_.param<double>("cmd_speed", cmd_speed_, 0.3);
        nh_.param<bool>("use_local_path_logic", use_local_path_logic_, true);
        nh_.param<double>("turnAroundThreshold", turnAroundThreshold_, 5.0);
        nh_.param<double>("stopYawRateGain", stopYawRateGain_, 7.5);
        nh_.param<double>("maxYawRate", maxYawRate_, 45.0);
        
        // Turn around return goal
        nh_.param<double>("return_goal_x", return_goal_x_, -9.685347557067871);
        nh_.param<double>("return_goal_y", return_goal_y_, 5.662903785705566);
        nh_.param<double>("return_goal_z", return_goal_z_, 0.0);
        nh_.param<double>("return_goal_qx", return_goal_qx_, 0.0);
        nh_.param<double>("return_goal_qy", return_goal_qy_, 0.0);
        nh_.param<double>("return_goal_qz", return_goal_qz_, 0.9795326031700423);
        nh_.param<double>("return_goal_qw", return_goal_qw_, 0.20128556661350672);

        // Convert threshold to radians
        turnAroundThreshold_ = turnAroundThreshold_ * M_PI / 180.0;

        // Subscribers
        sub_path_ = nh_public_.subscribe(path_topic, 10, &KeyCmdFollower::pathCallback, this);
        sub_odom_ = nh_public_.subscribe(odom_topic, 10, &KeyCmdFollower::odomCallback, this);
        sub_goal_ = nh_public_.subscribe("/move_base_simple/goal", 10, &KeyCmdFollower::goalPoseCallback, this);
        sub_turn_around_ = nh_public_.subscribe("/turn_around", 5, &KeyCmdFollower::turnAroundHandler, this);

        // Publishers
        pub_ctrl_cmd_ = nh_public_.advertise<yhs_can_msgs::ctrl_cmd>("/ctrl_cmd", 1);
        pub_marker_ = nh_public_.advertise<visualization_msgs::Marker>("/calculated_velocity", 10);
        pub_turn_around_status_ = nh_public_.advertise<std_msgs::Int8>("/turn_around_status", 5);
        pub_goal_return_ = nh_public_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);

        current_position_.x = 0;
        current_position_.y = 0;
    }

    void run()
    {
        ros::Rate rate(50);
        while (ros::ok())
        {
            ros::spinOnce();
            
            // Handle turn around with highest priority
            if (turnAroundInProgress_)
            {
                processTurnAround();
                rate.sleep();
                continue;
            }
            
            if (path_received_ && odom_received_)
            {
                updateControl();
            }
            rate.sleep();
        }
    }

private:
    struct Vec2 {
        double x, y;
    };

    ros::NodeHandle nh_public_;
    ros::NodeHandle nh_;

    ros::Subscriber sub_path_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_turn_around_;
    ros::Publisher pub_ctrl_cmd_;
    ros::Publisher pub_marker_;
    ros::Publisher pub_turn_around_status_;
    ros::Publisher pub_goal_return_;

    std::vector<Vec2> path_points_;
    Vec2 current_position_;
    double current_yaw_;
    
    bool path_received_;
    bool odom_received_;
    bool reached_goal_;
    double goal_yaw_;
    
    // Smoothing variables
    double real_velocity_;
    double real_steering_;

    double lookahead_index_offset_;
    double cmd_speed_;
    bool use_local_path_logic_;
    
    // Turn around variables
    bool turnAroundTriggered_;
    bool turnAroundInProgress_;
    bool turnAroundCompleted_;
    double turnAroundTargetYaw_;
    double turnAroundStartYaw_;
    double turnAroundThreshold_;
    double stopYawRateGain_;
    double maxYawRate_;
    
    // Return goal after turn around
    double return_goal_x_, return_goal_y_, return_goal_z_;
    double return_goal_qx_, return_goal_qy_, return_goal_qz_, return_goal_qw_;

    double InertialElement(double tao, double GiveValue, double Value_last_time)
    {
        return (tao * Value_last_time + GiveValue) / (1 + tao);
    }
    
    void turnAroundHandler(const std_msgs::Int8::ConstPtr& trigger)
    {
        if (trigger->data == 1 && !turnAroundInProgress_)
        {
            turnAroundTriggered_ = true;
            turnAroundStartYaw_ = current_yaw_;
            
            // Calculate target yaw (current + 180 degrees)
            turnAroundTargetYaw_ = current_yaw_ + M_PI;
            
            // Normalize to [-π, π]
            if (turnAroundTargetYaw_ > M_PI)
                turnAroundTargetYaw_ -= 2 * M_PI;
            else if (turnAroundTargetYaw_ < -M_PI)
                turnAroundTargetYaw_ += 2 * M_PI;
            
            turnAroundInProgress_ = true;
            ROS_INFO("Turn around triggered: current yaw = %.2f, target yaw = %.2f", 
                     current_yaw_ * 180.0 / M_PI, turnAroundTargetYaw_ * 180.0 / M_PI);
        }
    }
    
    void processTurnAround()
    {
        // Calculate yaw difference
        double yawDiff = turnAroundTargetYaw_ - current_yaw_;
        
        // Normalize to [-π, π]
        while (yawDiff > M_PI) yawDiff -= 2 * M_PI;
        while (yawDiff < -M_PI) yawDiff += 2 * M_PI;

        // Check if turn around is complete
        if (std::abs(yawDiff) < turnAroundThreshold_)
        {
            turnAroundInProgress_ = false;
            turnAroundTriggered_ = false;
            turnAroundCompleted_ = true;
            
            // Stop the vehicle
            real_velocity_ = 0.0;
            real_steering_ = 0.0;
            
            yhs_can_msgs::ctrl_cmd ctrl_cmd_msg;
            ctrl_cmd_msg.ctrl_cmd_gear = 6;
            ctrl_cmd_msg.ctrl_cmd_x_linear = 0.0;
            ctrl_cmd_msg.ctrl_cmd_z_angular = 0.0;
            pub_ctrl_cmd_.publish(ctrl_cmd_msg);

            // Publish completion status
            std_msgs::Int8 status_msg;
            status_msg.data = 1;  // 1 = completed
            pub_turn_around_status_.publish(status_msg);

            // Publish return goal
            geometry_msgs::PoseStamped goalMsg;
            goalMsg.header.stamp = ros::Time::now();
            goalMsg.header.frame_id = "map:start";
            goalMsg.pose.position.x = return_goal_x_;
            goalMsg.pose.position.y = return_goal_y_;
            goalMsg.pose.position.z = return_goal_z_;
            goalMsg.pose.orientation.x = return_goal_qx_;
            goalMsg.pose.orientation.y = return_goal_qy_;
            goalMsg.pose.orientation.z = return_goal_qz_;
            goalMsg.pose.orientation.w = return_goal_qw_;
            pub_goal_return_.publish(goalMsg);
            
            ROS_INFO("Turn around completed! Final yaw error: %.2f degrees", 
                     std::abs(yawDiff) * 180.0 / M_PI);
            return;
        }

        // Execute in-place rotation
        // Calculate angular velocity based on yaw difference
        double turnYawRate = stopYawRateGain_ * yawDiff;
        
        // Limit angular velocity
        double maxYawRateRad = maxYawRate_ * M_PI / 180.0;
        if (turnYawRate > maxYawRateRad)
            turnYawRate = maxYawRateRad;
        else if (turnYawRate < -maxYawRateRad)
            turnYawRate = -maxYawRateRad;

        // Publish command
        yhs_can_msgs::ctrl_cmd ctrl_cmd_msg;
        ctrl_cmd_msg.ctrl_cmd_gear = 6;
        ctrl_cmd_msg.ctrl_cmd_x_linear = 0.0;  // No linear velocity
        ctrl_cmd_msg.ctrl_cmd_z_angular = turnYawRate * 180.0 / M_PI;  // Convert to degrees
        pub_ctrl_cmd_.publish(ctrl_cmd_msg);
        
        ROS_INFO_THROTTLE(1.0, "Turning... current: %.2f, target: %.2f, diff: %.2f degrees", 
                          current_yaw_ * 180.0 / M_PI, turnAroundTargetYaw_ * 180.0 / M_PI, 
                          yawDiff * 180.0 / M_PI);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        path_points_.clear();
        for (const auto& pose : msg->poses)
        {
            path_points_.push_back({pose.pose.position.x, pose.pose.position.y});
        }
        if (!path_points_.empty()) {
            path_received_ = true;
            reached_goal_ = false;
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        current_position_.x = msg->pose.pose.position.x;
        current_position_.y = msg->pose.pose.position.y;

        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        tf::Quaternion quat(qx, qy, qz, qw);
        tf::Matrix3x3 mat(quat);
        double roll, pitch;
        mat.getRPY(roll, pitch, current_yaw_);
        
        odom_received_ = true;
    }

    void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        double qx = msg->pose.orientation.x;
        double qy = msg->pose.orientation.y;
        double qz = msg->pose.orientation.z;
        double qw = msg->pose.orientation.w;
        
        tf::Quaternion quat(qx, qy, qz, qw);
        tf::Matrix3x3 mat(quat);
        double roll, pitch;
        mat.getRPY(roll, pitch, goal_yaw_);
        
        ROS_INFO("Received goal yaw: %.2f", goal_yaw_);
    }

    void updateControl()
    {
        if (path_points_.empty()) return;

        double tracking_pos_x = current_position_.x;
        double tracking_pos_y = current_position_.y;
        double tracking_yaw = current_yaw_;

        if (use_local_path_logic_) {
            tracking_pos_x = 0.0;
            tracking_pos_y = 0.0;
            tracking_yaw = 0.0;
        }

        // Find closest point on path
        double min_distance = std::numeric_limits<double>::max();
        size_t closest_index = 0;
        size_t size = path_points_.size();

        for (size_t i = 0; i < size; i++)
        {
            double dx = path_points_[i].x - tracking_pos_x;
            double dy = path_points_[i].y - tracking_pos_y;
            double distance = std::sqrt(dx * dx + dy * dy);
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_index = i;
            }
        }

        // Distance to the end of the path (using the chosen coordinate system)
        double end_dx = path_points_.back().x - tracking_pos_x;
        double end_dy = path_points_.back().y - tracking_pos_y;
        double end_distance = std::sqrt(end_dx * end_dx + end_dy * end_dy);

        // ---------- Goal Alignment Logic ----------
        if (end_distance <= 0.2) // Increased slightly from 0.1 to 0.2
        {
            reached_goal_ = true;
        }

        if (reached_goal_)
        {
            // Calculate yaw error (Global yaw is always used for final alignment because Goal is Global!)
            double yaw_error = goal_yaw_ - current_yaw_;
            
            // Normalize yaw error to [-pi, pi]
            while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
            
            double yaw_error_deg = yaw_error * 180.0 / M_PI;

            // If aligned within threshold, stop
            if (std::abs(yaw_error_deg) < 5.0) 
            {
                stopCar();
                reached_goal_ = false; // Or keep it true if you want to stay in "reached" state
                return;
            }

            // Align in place
            // Target Linear Speed: 0.01 (Creep), Angular: yaw_error_deg
            double target_velocity = 0.01;
            double target_steering = yaw_error_deg;

            // Apply smoothing
            real_velocity_ = InertialElement(8, target_velocity, real_velocity_);
            real_steering_ = InertialElement(3, target_steering, real_steering_);

            yhs_can_msgs::ctrl_cmd ctrl_cmd_msg;
            ctrl_cmd_msg.ctrl_cmd_gear = 6;
            ctrl_cmd_msg.ctrl_cmd_x_linear = real_velocity_;
            ctrl_cmd_msg.ctrl_cmd_z_angular = real_steering_; 

            pub_ctrl_cmd_.publish(ctrl_cmd_msg);
            // ROS_INFO_THROTTLE(1.0, "Aligning yaw... error=%.2f", yaw_error_deg);
            return;
        }

        // 2. Select lookahead point
        size_t dot_next_index = closest_index + (size_t)lookahead_index_offset_;
        if (dot_next_index >= size)
            dot_next_index = size - 1;

        // 3. Compute vector to lookahead point
        Vec2 next_pt = path_points_[dot_next_index];
        Vec2 theta_vector = {next_pt.x - tracking_pos_x, next_pt.y - tracking_pos_y};

        // 4. Compute vehicle heading vector
        Vec2 yaw_vector = {std::cos(tracking_yaw), std::sin(tracking_yaw)};

        // 5. Compute angle using dot product
        // theta_vector dot yaw_vector
        double dot_product = yaw_vector.x * theta_vector.x + yaw_vector.y * theta_vector.y;
        double mag_a = std::sqrt(yaw_vector.x * yaw_vector.x + yaw_vector.y * yaw_vector.y);
        double mag_b = std::sqrt(theta_vector.x * theta_vector.x + theta_vector.y * theta_vector.y);
        
        double cos_theta = 0.0;
        if (mag_a * mag_b > 1e-6) {
            cos_theta = dot_product / (mag_a * mag_b);
        }
        
        // Clamp
        if (cos_theta > 1.0) cos_theta = 1.0;
        if (cos_theta < -1.0) cos_theta = -1.0;

        double theta = std::acos(cos_theta);

        // 6. Determine direction using cross product (z component of 2D cross)
        // cross = ax * by - ay * bx
        double cross_product_z = yaw_vector.x * theta_vector.y - yaw_vector.y * theta_vector.x;
        if (cross_product_z < 0) {
            theta = -theta;
        }

        // Convert to degrees
        double theta_deg = theta * 180.0 / M_PI;

        // 7. Publish Command
        double target_velocity = cmd_speed_;
        double target_steering = theta_deg;

        // Apply smoothing
        real_velocity_ = InertialElement(8, target_velocity, real_velocity_);
        real_steering_ = InertialElement(3, target_steering, real_steering_);

        yhs_can_msgs::ctrl_cmd ctrl_cmd_msg;
        ctrl_cmd_msg.ctrl_cmd_gear = 6; 
        ctrl_cmd_msg.ctrl_cmd_x_linear = real_velocity_;
        ctrl_cmd_msg.ctrl_cmd_z_angular = real_steering_; 

        pub_ctrl_cmd_.publish(ctrl_cmd_msg);

        // Publish marker
        // NOTE: If using local logic, we need to transform vector back to global for visualization
        // Or just publish in body frame? Marker header is usually 'map'. 
        // For simplicity, visualizing in map frame with odom data is better.
        // But if local logic is used, theta_vector is local.
        // Let's visualize based on actual vehicle position in Map always.

        double viz_vel_x, viz_vel_y;
        if (use_local_path_logic_) {
            // theta_vector is in body frame (x forward, y left).
            // Need to rotate it by current_yaw_ to get map frame vector.
            viz_vel_x = std::cos(current_yaw_) * theta_vector.x - std::sin(current_yaw_) * theta_vector.y;
            viz_vel_y = std::sin(current_yaw_) * theta_vector.x + std::cos(current_yaw_) * theta_vector.y;
        } else {
            viz_vel_x = theta_vector.x;
            viz_vel_y = theta_vector.y;
        }
        
        publishVector(current_position_.x, current_position_.y, viz_vel_x, viz_vel_y);
    }
    
    void stopCar()
    {
        // For stop, we can reset the smoothing or let it decay. 
        // TeleopCar just sets cmd to 0. InertialElement would decay it.
        // But stopCar implies immediate stop usually.
        // Let's set target to 0 and loop, OR force 0.
        // TeleopCar's stopCar sets cmd_velocity_ = 0.
        // But InertialElement loop continues to run.
        // Here we don't have a background loop if we exit run(), but inside updateControl we are looping.
        // If we want Immediate Stop (Emergency/Done), we can force variables to 0.
        
        real_velocity_ = 0.0;
        real_steering_ = 0.0;

        yhs_can_msgs::ctrl_cmd ctrl_cmd_msg;
        ctrl_cmd_msg.ctrl_cmd_gear = 6;
        ctrl_cmd_msg.ctrl_cmd_x_linear = 0.0;
        ctrl_cmd_msg.ctrl_cmd_z_angular = 0.0;
        pub_ctrl_cmd_.publish(ctrl_cmd_msg);
    }

    void publishVector(double current_posx, double current_posy, double velocity_x, double velocity_y)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "velocity_vector";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = current_posx;
        marker.pose.position.y = current_posy;
        marker.pose.position.z = 0.0;

        double arrow_yaw = std::atan2(velocity_y, velocity_x);
        tf::Quaternion arrow_quat = tf::createQuaternionFromRPY(0, 0, arrow_yaw);
        tf::quaternionTFToMsg(arrow_quat, marker.pose.orientation);

        marker.scale.x = std::sqrt(velocity_x*velocity_x + velocity_y*velocity_y); 
        // Cap visual scale if needed, or leave it proportional to distance
        if (marker.scale.x > 2.0) marker.scale.x = 2.0;

        marker.scale.y = 0.1; 
        marker.scale.z = 0.1; 

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        pub_marker_.publish(marker);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "key_cmd_follower");
    KeyCmdFollower follower;
    follower.run();
    return 0;
}