#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "yhs_can_msgs/ctrl_cmd.h"

using namespace std;

const double PI = 3.1415926;
static std::string ODOM_TOPIC;  //将状态估计信息话题写为参数在launch中加载
double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;           // 发布次数
int pubSkipCount = 0;         // 发布次数计数器
bool twoWayDrive = true;      // 双向驱动
double lookAheadDis = 0.5;    // 向前搜索的路点距离
double lookAheadSpeedGain = 0.0;
double yawRateGain = 7.5;     // 普通的yaw率增益
double stopYawRateGain = 7.5; // 停止的yaw率增益
double maxYawRate = 45.0;     // 最大旋转速度
bool useYawRateFilter = true;
double yawRateFilterAlpha = 0.3;
bool useYawRateRateLimit = true;
double yawRateMaxDelta = 10.0;
double maxSpeed = 1.0;        // 最大直线速度
double maxAccel = 1.0;        // 最大加速度
double switchTimeThre = 1.0;
double dirDiffThre = 0.1;       // 方向差阈值
double dirDiffGoThre = 0.08;    // 小于该阈值才允许前进
double dirDiffStopThre = 0.18;  // 大于该阈值才强制停止
double dirDiffTurnOnlyThre = 0.45; // 大于该阈值只原地转向
double turnOnlyDistThre = 0.6;  // 小于该距离才允许纯原地转向
double stopDisThre = 0.2;       // 停止距离阈值
double slowDwnDisThre = 1.0;    // 减速距离阈值
double posOnlyDistThre = 0.5;   // 小于该距离仅做角度调整
double nearGoalNoSwitchDist = 2.5; // 近目标不切换前后方向
double moveWhileTurningDist = 3.0; // 距离较远时允许边转边走
bool useInclRateToSlow = false; // 使用倾斜速率来减速
double inclRateThre = 120.0;    // 倾斜速率阈值
double slowRate1 = 0.25;        // 减速率1
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false; // 使用倾斜度来停止
double inclThre = 45.0;     // 倾斜度阈值
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;
double speedFilterAlpha = 0.3;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
int safetyStop = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
double switchTime = 0;
double lastYawRateCmd = 0.0;
bool pathJustUpdated = false;
bool headingAligned = false;
double lastTargetSpeed = 0.0;
double lastPublishedLinear = 0.0;


// 掉头相关
bool turnAroundTriggered = false;  // 是否触发原地掉头
double turnAroundTargetYaw = 0.0;  // 掉头目标角度
double turnAroundStartYaw = 0.0;   // 掉头开始时的角度
bool turnAroundInProgress = false; // 正在执行掉头
bool turnAroundCompleted = false;  // 掉头完成标志
double turnAroundThreshold = 5.0 * PI / 180.0; 
void processTurnAround(ros::Publisher& pubSpeed_1, ros::Publisher& pubSpeed, ros::Publisher& yhs_ctrl_pub, 
                       geometry_msgs::TwistStamped& cmd_vel_1, geometry_msgs::Twist& cmd_vel);


nav_msgs::Path path;

void odomHandler(const nav_msgs::Odometry::ConstPtr &odomIn)
{
  odomTime = odomIn->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;

  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop)
  {
    stopInitTime = odomIn->header.stamp.toSec();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow)
  {
    slowInitTime = odomIn->header.stamp.toSec();
  }
}

void turnAroundHandler(const std_msgs::Int8::ConstPtr &trigger)
{
  if (trigger->data == 1 && !turnAroundInProgress)
  {
    turnAroundTriggered = true;
    turnAroundStartYaw = vehicleYaw;
    // 计算目标角度(当前角度+180度)
    turnAroundTargetYaw = vehicleYaw + PI;
    // 归一化到[-π, π]
    if (turnAroundTargetYaw > PI)
      turnAroundTargetYaw -= 2 * PI;
    else if (turnAroundTargetYaw < -PI)
      turnAroundTargetYaw += 2 * PI;
    
    turnAroundInProgress = true;
    ROS_INFO("Turn around triggered: current yaw = %.2f, target yaw = %.2f", 
             vehicleYaw * 180.0 / PI, turnAroundTargetYaw * 180.0 / PI);
  }
}


void pathHandler(const nav_msgs::Path::ConstPtr &pathIn)
{
  int pathSize = pathIn->poses.size();
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++)
  {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
  pathJustUpdated = true;
}

void speedHandler(const std_msgs::Float32::ConstPtr &speed)
{
  double speedTime = ros::Time::now().toSec();

  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0)
  {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0)
      joySpeed = 0;
    else if (joySpeed > 1.0)
      joySpeed = 1.0;
  }
}

void stopHandler(const std_msgs::Int8::ConstPtr &stop)
{
  safetyStop = stop->data;
}


void processTurnAround(ros::Publisher& pubSpeed_1, ros::Publisher& pubSpeed, ros::Publisher& yhs_ctrl_pub,
                       geometry_msgs::TwistStamped& cmd_vel_1, geometry_msgs::Twist& cmd_vel,
                       ros::Publisher& pubTurnAroundStatus)
{
  // 计算当前角度与目标角度的差值
  float yawDiff = turnAroundTargetYaw - vehicleYaw;
  
  // 归一化角度差到[-π, π]
  if (yawDiff > PI)
    yawDiff -= 2 * PI;
  else if (yawDiff < -PI)
    yawDiff += 2 * PI;

  // 判断是否完成掉头
  if (fabs(yawDiff) < turnAroundThreshold)
  {
    turnAroundInProgress = false;
    turnAroundTriggered = false;
    turnAroundCompleted = true;
    vehicleSpeed = 0;
    vehicleYawRate = 0;
    
    // 发布停止指令
    cmd_vel_1.header.stamp = ros::Time().fromSec(odomTime);
    cmd_vel_1.twist.linear.x = 0;
    cmd_vel_1.twist.angular.z = 0;

    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;

    yhs_can_msgs::ctrl_cmd yhs_cmd_vel;
    yhs_cmd_vel.ctrl_cmd_gear = 6;
    yhs_cmd_vel.ctrl_cmd_x_linear = 0;
    yhs_cmd_vel.ctrl_cmd_z_angular = 0;
    yhs_ctrl_pub.publish(yhs_cmd_vel);

    pubSpeed_1.publish(cmd_vel_1);
    pubSpeed.publish(cmd_vel);

    std_msgs::Int8 status_msg;
    status_msg.data = 1;  // 1表示完成
    pubTurnAroundStatus.publish(status_msg);
    
    ROS_INFO("Turn around completed! Final yaw error: %.2f degrees", fabs(yawDiff) * 180.0 / PI);
    return;
  }

  // 执行原地旋转
  vehicleSpeed = 0;  // 线速度为0
  
  // 根据角度差计算角速度
  double turnYawRate = stopYawRateGain * yawDiff;
  
  // 限制角速度
  if (turnYawRate > maxYawRate * PI / 180.0)
    turnYawRate = maxYawRate * PI / 180.0;
  else if (turnYawRate < -maxYawRate * PI / 180.0)
    turnYawRate = -maxYawRate * PI / 180.0;
  
  vehicleYawRate = turnYawRate;

  // 发布速度指令
  pubSkipCount--;
  if (pubSkipCount < 0)
  {
    cmd_vel_1.header.stamp = ros::Time().fromSec(odomTime);
    cmd_vel_1.twist.linear.x = 0;
    cmd_vel_1.twist.angular.z = vehicleYawRate;

    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = vehicleYawRate;

    yhs_can_msgs::ctrl_cmd yhs_cmd_vel;
    yhs_cmd_vel.ctrl_cmd_gear = 6;
    yhs_cmd_vel.ctrl_cmd_x_linear = 0;
    yhs_cmd_vel.ctrl_cmd_z_angular = vehicleYawRate * 57.29f;
    yhs_ctrl_pub.publish(yhs_cmd_vel);

    pubSpeed_1.publish(cmd_vel_1);
    pubSpeed.publish(cmd_vel);

    pubSkipCount = pubSkipNum;
    
    ROS_INFO_THROTTLE(1.0, "Turning... current: %.2f, target: %.2f, diff: %.2f degrees", 
                      vehicleYaw * 180.0 / PI, turnAroundTargetYaw * 180.0 / PI, yawDiff * 180.0 / PI);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pathFollower");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("pubSkipNum", pubSkipNum);
  nhPrivate.getParam("twoWayDrive", twoWayDrive);
  nhPrivate.getParam("lookAheadDis", lookAheadDis);
  nhPrivate.getParam("lookAheadSpeedGain", lookAheadSpeedGain);
  nhPrivate.getParam("yawRateGain", yawRateGain);
  nhPrivate.getParam("stopYawRateGain", stopYawRateGain);
  nhPrivate.getParam("maxYawRate", maxYawRate);
  nhPrivate.getParam("useYawRateFilter", useYawRateFilter);
  nhPrivate.getParam("yawRateFilterAlpha", yawRateFilterAlpha);
  nhPrivate.getParam("useYawRateRateLimit", useYawRateRateLimit);
  nhPrivate.getParam("yawRateMaxDelta", yawRateMaxDelta);
  nhPrivate.getParam("maxSpeed", maxSpeed);
  nhPrivate.getParam("maxAccel", maxAccel);
  nhPrivate.getParam("switchTimeThre", switchTimeThre);
  nhPrivate.getParam("dirDiffThre", dirDiffThre);
  nhPrivate.getParam("dirDiffGoThre", dirDiffGoThre);
  nhPrivate.getParam("dirDiffStopThre", dirDiffStopThre);
  nhPrivate.getParam("dirDiffTurnOnlyThre", dirDiffTurnOnlyThre);
  nhPrivate.getParam("turnOnlyDistThre", turnOnlyDistThre);
  nhPrivate.getParam("stopDisThre", stopDisThre);
  nhPrivate.getParam("slowDwnDisThre", slowDwnDisThre);
  nhPrivate.getParam("posOnlyDistThre", posOnlyDistThre);
  nhPrivate.getParam("nearGoalNoSwitchDist", nearGoalNoSwitchDist);
  nhPrivate.getParam("moveWhileTurningDist", moveWhileTurningDist);
  nhPrivate.getParam("useInclRateToSlow", useInclRateToSlow);
  nhPrivate.getParam("inclRateThre", inclRateThre);
  nhPrivate.getParam("slowRate1", slowRate1);
  nhPrivate.getParam("slowRate2", slowRate2);
  nhPrivate.getParam("slowTime1", slowTime1);
  nhPrivate.getParam("slowTime2", slowTime2);
  nhPrivate.getParam("useInclToStop", useInclToStop);
  nhPrivate.getParam("inclThre", inclThre);
  nhPrivate.getParam("stopTime", stopTime);
  nhPrivate.getParam("noRotAtStop", noRotAtStop);
  nhPrivate.getParam("noRotAtGoal", noRotAtGoal);
  nhPrivate.getParam("autonomyMode", autonomyMode);
  nhPrivate.getParam("autonomySpeed", autonomySpeed);
  nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);
  nhPrivate.getParam("speedFilterAlpha", speedFilterAlpha);
  nhPrivate.getParam("odom_topic", ODOM_TOPIC);  //状态估计话题
  nhPrivate.getParam("turnAroundThreshold", turnAroundThreshold);// 原地掉头话题， 角度单位
  
  

  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>(ODOM_TOPIC, 5, odomHandler);

  ros::Subscriber subPath = nh.subscribe<nav_msgs::Path>("/local_path", 5, pathHandler);

  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32>("/speed", 5, speedHandler);

  ros::Subscriber subStop = nh.subscribe<std_msgs::Int8>("/stop", 5, stopHandler);

  ros::Subscriber subTurnAround = nh.subscribe<std_msgs::Int8>("/turn_around", 5, turnAroundHandler);

  ros::Publisher pubSpeed_1 = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel_1", 5);
  ros::Publisher pubSpeed = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
  geometry_msgs::Twist cmd_vel;
  ros::Publisher yhs_ctrl_pub = nh.advertise<yhs_can_msgs::ctrl_cmd>("/ctrl_cmd", 1);

  turnAroundThreshold = turnAroundThreshold * PI / 180.0;// 转成弧度
  ros::Publisher pubTurnAroundStatus = nh.advertise<std_msgs::Int8>("/turn_around_status", 5);
  
  geometry_msgs::TwistStamped cmd_vel_1;
  cmd_vel_1.header.frame_id = "body";

  if (autonomyMode)
  {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0)
      joySpeed = 0;
    else if (joySpeed > 1.0)
      joySpeed = 1.0;
  }

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();

    // 处理原地掉头 
    if (turnAroundInProgress)
    {
      processTurnAround(pubSpeed_1, pubSpeed, yhs_ctrl_pub, cmd_vel_1, cmd_vel, pubTurnAroundStatus);
      status = ros::ok();
      rate.sleep();
      continue;  
    }

    if (pathInit)
    {
      // 计算相对目标车辆的x和y距离
      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

      // 获取路径点的数量
      int pathSize = path.poses.size();
      float effectiveLookAhead = lookAheadDis + lookAheadSpeedGain * fabs(vehicleSpeed);
      if (effectiveLookAhead < 0.1)
        effectiveLookAhead = 0.1;

      if (pathJustUpdated && pathSize > 0)
      {
        float bestDis = 1e9;
        int bestIdx = 0;
        for (int i = 0; i < pathSize; i++)
        {
          float dx = path.poses[i].pose.position.x - vehicleXRel;
          float dy = path.poses[i].pose.position.y - vehicleYRel;
          float d = sqrt(dx * dx + dy * dy);
          if (d < bestDis)
          {
            bestDis = d;
            bestIdx = i;
          }
        }
        pathPointID = bestIdx;
        pathJustUpdated = false;
      }

      // 计算路径终点与当前车辆的x和y距离
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

      float disX, disY, dis;
      // 在路径点中循环，直到找到一个距离大于lookAheadDis的路径点
      while (pathPointID < pathSize - 1)
      {
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        // 如果当前路径点距离小于lookAheadDis，则继续向下一个路径点移动
        if (dis < effectiveLookAhead)
        {
          pathPointID++;
        }
        else
        {
          break;
        }
      }

      // 计算车辆相对于路径点的x和y距离
      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      // 计算距离
      dis = sqrt(disX * disX + disY * disY);

      // 计算路径的方向
      float pathDir = atan2(disY, disX);

      // 计算车辆方向和路径方向的差值
      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;

      // 如果差值大于π，则减去2π，如果差值小于-π，则加上2π，以保证差值在[-π, π]范围内
      if (dirDiff > PI)
        dirDiff -= 2 * PI;
      else if (dirDiff < -PI)
        dirDiff += 2 * PI;

      // 再次计算差值，如果差值大于π，则减去2π，如果差值小于-π，则加上2π，以保证差值在[-π, π]范围内
      if (dirDiff > PI)
        dirDiff -= 2 * PI;
      else if (dirDiff < -PI)
        dirDiff += 2 * PI;


      // 如果车辆是双向驾驶并且方向差超过±π/2并且当前速度大于0，则切换车辆的行驶方向，并记录切换的时间
      bool allowDirSwitch = true;
      if (endDis < nearGoalNoSwitchDist && fabs(dirDiff) < PI / 2)
        allowDirSwitch = false;

      if (twoWayDrive && allowDirSwitch)
      {
        double time = ros::Time::now().toSec();
        if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre)
        {
          navFwd = false;
          switchTime = time;
        }
        else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre)
        {
          navFwd = true;
          switchTime = time;
        }
      }

      // 根据车辆的行驶方向调整速度
      // 计算基于最大速度和操纵杆输入的速度
      float joySpeed2 = maxSpeed * joySpeed;

      // 如果车辆不是向前行驶
      if (!navFwd)
      {
        // 增加方向差
        dirDiff += PI;
        // 如果方向差大于PI，减去2*PI
        if (dirDiff > PI)
          dirDiff -= 2 * PI;
        // 改变速度的方向
        joySpeed2 *= -1;
      }

      if (!headingAligned && fabs(dirDiff) < dirDiffGoThre)
        headingAligned = true;
      else if (headingAligned && fabs(dirDiff) > dirDiffStopThre)
        headingAligned = false;

      double rawYawRate = 0.0;
      // 如果车辆速度小于2倍最大加速度
      if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0)
        // 设置yaw率为停止的yaw率增益乘以方向差
        rawYawRate = -stopYawRateGain * dirDiff;
      else
        // 设置yaw率为普通的yaw率增益乘以方向差
        rawYawRate = -yawRateGain * dirDiff;

      // 如果yaw率大于最大yaw率，设置为最大yaw率
      if (rawYawRate > maxYawRate * PI / 180.0)
        rawYawRate = maxYawRate * PI / 180.0;
      // 如果yaw率小于最小yaw率，设置为最小yaw率
      else if (rawYawRate < -maxYawRate * PI / 180.0)
        rawYawRate = -maxYawRate * PI / 180.0;

      if (useYawRateFilter)
        rawYawRate = yawRateFilterAlpha * rawYawRate + (1.0 - yawRateFilterAlpha) * lastYawRateCmd;

      if (useYawRateRateLimit)
      {
        double maxDelta = yawRateMaxDelta * PI / 180.0;
        double delta = rawYawRate - lastYawRateCmd;
        if (delta > maxDelta)
          rawYawRate = lastYawRateCmd + maxDelta;
        else if (delta < -maxDelta)
          rawYawRate = lastYawRateCmd - maxDelta;
      }

      vehicleYawRate = rawYawRate;

      // 如果操纵杆速度为0且不在自主模式
      if (joySpeed2 == 0 && !autonomyMode)
      {
        // 设置yaw率为最大yaw率乘以操纵杆的yaw输入
        vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;
      }
      // 如果路径大小小于等于1或距离小于停止距离且不旋转到目标（算法根据接收到的路径的路径点数量pathSize来判断是否到达目标点）
      else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal))
      {
        // 设置yaw率为0
        vehicleYawRate = 0;
      }

      lastYawRateCmd = vehicleYawRate;

      // 如果路径大小小于等于1
      if (pathSize <= 1)
      {
        // 设置速度为0
        joySpeed2 = 0;
      }
      // 否则如果距离除以减速距离小于操纵杆速度
      else if (endDis / slowDwnDisThre < joySpeed)
      {
        // 减少速度
        joySpeed2 *= endDis / slowDwnDisThre;
      }

      // 创建一个新的变量joySpeed3，用于存储joySpeed2的值
      float joySpeed3 = joySpeed2;
      // 如果当前时间小于慢速初始时间加上慢速时间1，并且慢速初始时间大于0
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0)
      {
        // 将joySpeed3乘以慢速率1，以降低速度
        joySpeed3 *= slowRate1;
      }
      // 如果当前时间小于慢速初始时间加上慢速时间1加上慢速时间2，并且慢速初始时间大于0
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0)
      {
        // 将joySpeed3乘以慢速率2，以降低速度
        joySpeed3 *= slowRate2;
      }
      // 如果方向差绝对值小于方向差阈值，并且距离大于停止距离
      double targetSpeed = joySpeed3;
      if (fabs(dirDiff) > dirDiffTurnOnlyThre && dis < turnOnlyDistThre)
        targetSpeed = 0.0;
      else
      {
        double dirGain = cos(dirDiff);
        if (!twoWayDrive && dirGain < 0.0)
          dirGain = 0.0;
        targetSpeed *= dirGain;
      }

      if (dis < posOnlyDistThre)
        targetSpeed = 0.0;

      targetSpeed = speedFilterAlpha * targetSpeed + (1.0 - speedFilterAlpha) * lastTargetSpeed;
      lastTargetSpeed = targetSpeed;

      bool allowMove = headingAligned || dis > moveWhileTurningDist;
      if (allowMove && dis > stopDisThre)
      {
        // 如果车辆速度小于joySpeed3，加速
        if (vehicleSpeed < targetSpeed)
          vehicleSpeed += maxAccel / 100.0;
        // 如果车辆速度大于joySpeed3，减速
        else if (vehicleSpeed > targetSpeed)
          vehicleSpeed -= maxAccel / 100.0;
      }
      // 否则
      else
      {
        // 如果车辆速度大于0，减速
        if (vehicleSpeed > 0)
          vehicleSpeed -= maxAccel / 100.0;
        // 如果车辆速度小于0，加速
        else if (vehicleSpeed < 0)
          vehicleSpeed += maxAccel / 100.0;
      }

      // 如果当前时间小于停止初始时间加上停止时间，并且停止初始时间大于0
      if (odomTime < stopInitTime + stopTime && stopInitTime > 0)
      {
        // 将车辆速度设置为0
        vehicleSpeed = 0;
        // 将yaw率设置为0
        vehicleYawRate = 0;
      }

      // 如果安全停止级别大于等于1
      if (safetyStop >= 1)
      {
        // 将车辆速度设置为0
        vehicleSpeed = 0;
      }
      // 如果安全停止级别大于等于2
      if (safetyStop >= 2)
      {
        // 将yaw率设置为0
        vehicleYawRate = 0;
      }

      // 减少发布次数计数器
      pubSkipCount--;
      // 如果发布次数计数器小于0
      if (pubSkipCount < 0)
      {
        double zeroSpeedEps = maxAccel / 100.0;
        double publishLinear = vehicleSpeed;
        if (fabs(vehicleSpeed) <= zeroSpeedEps)
        {
          publishLinear = 0.0;
        }

        double publishYaw = vehicleYawRate;
        bool leavingStop = fabs(lastPublishedLinear) <= zeroSpeedEps && fabs(publishLinear) > zeroSpeedEps;
        if (leavingStop)
        {
          // 起步时强制角速度为0，避免带角度起步
          publishYaw = 0.0;
        }
        vehicleYawRate = publishYaw;
        lastYawRateCmd = publishYaw;

        // 设置命令速度消息的时间戳为当前时间
        cmd_vel_1.header.stamp = ros::Time().fromSec(odomTime);
        cmd_vel_1.twist.linear.x = publishLinear;
        cmd_vel_1.twist.angular.z = publishYaw;

        cmd_vel.linear.x = publishLinear;
        cmd_vel.angular.z = publishYaw;

        yhs_can_msgs::ctrl_cmd yhs_cmd_vel;
        yhs_cmd_vel.ctrl_cmd_gear = 6;
        yhs_cmd_vel.ctrl_cmd_x_linear = publishLinear;    // 线速度
        yhs_cmd_vel.ctrl_cmd_z_angular = publishYaw * 57.29f;   // 角速度 (rad/s)
        yhs_ctrl_pub.publish(yhs_cmd_vel);

        // 发布命令速度消息
        pubSpeed_1.publish(cmd_vel_1);
	      pubSpeed.publish(cmd_vel);

        lastPublishedLinear = publishLinear;
        // 将发布次数计数器重置为发布次数数量
        pubSkipCount = pubSkipNum;
      }
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
