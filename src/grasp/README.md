# Project 2: Visual Grasp


## 1. Yolo module Setup
Use Yolo to detect object
> pip install ultralytics

test: 
> python test_yolo.py
 
## 2. Realsense Setup

**Realsense 一定要插到主板usb3.0接口**

> pip install pyrealsense2

test:
> python test_realsense.py
> 

## 3. Depth map to PointCloud

> pip install -r requirements
> 

terminal 1
> source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
> 
> python test_depth_2_pointcloud.py
> 
注意：将路径改为你自己的路径地址

terminal 2: RVIZ Visualization in camera frame
> source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
> 
> rosrun rviz rviz -d /home/ubuntu/arm_student_ws/project2/config/depth_point_visual.rviz 
> 
在RVIZ中，你会看到点云3D物体点云


## 4. Realsense (1) + Yolo (2) + Depth2PointCloud (3) + ROI Extraction

将上述模块组合起来

terminal 3: 
> source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
> 
> python realsense_yolo_pc_roi.py

在RVIZ中，你会看到红色的待抓取目标点

## 5. Execute Grasping

**提示：启动机械臂需要需要激活can模块**

terminal 4: publish TF
> source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
> 
> python piper_tf_publisher.py


terminal 5: RVIZ Visualization in world frame
> source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
> 
> roslaunch launch/piper_control.launch
>

注意：修改lanuch文件里的路径为你自己的路径地址

terminal 6:
> source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
> 
> python grasp_action.py
> 

## 6. (Optional) Hand-Eye calibration
如果抓取位置偏差过大，使用hand-eye calibration，标定相机内参

固定放置棋盘格，量取棋盘格左上角角点到机械臂基座中心的位移，填入 hand_eye_calibration.py 文件上方 

例如：world_t_chess = [0.23, 0, 0]

terminal 7：
> source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
> 
> python hand_eye_calibration.py
> 

terminal 8：
> source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
> 
> python piper_tf_publisher.py
> 

terminal 9： RVIZ visualization

> rosrun rviz rviz -d /home/ubuntu/arm_student_ws/project2/config/hand_eye_calibration.rviz
>

查看terminal1中的输出  link6_T_cam 即为标定结果，填入piper_arm.py 的 self.link6_q_camera 和 self.link6_t_camera中
