#!/bin/bash

# 打印欢迎信息
echo "====================================="
echo "    启动P3DX机器人DWA导航与数据记录    "
echo "====================================="

# 设置ROS环境
source ~/quad/navigation/catkin_MRPB/devel/setup.bash

# 确保目录存在
mkdir -p ~/quad/navigation/catkin_MRPB/src/dwa_planner/logs

# 先启动SLAM建图
# echo "启动SLAM建图..."
# roslaunch gmapping slam_gmapping_p3dx.launch &
# sleep 3  # 等待SLAM节点完全启动

# 启动导航系统（带数据记录）
echo "启动导航系统..."
roslaunch dwa_planner move_base_p3dx_dwa.launch