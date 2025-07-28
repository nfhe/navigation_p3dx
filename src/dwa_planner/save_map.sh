#!/bin/bash
# 文件名: save_map.sh

# 设置ROS环境
source ~/quad/navigation/catkin_MRPB/devel/setup.bash

# 创建保存目录
MAP_DIR="/home/he/quad/navigation/catkin_MRPB/src/dwa_planner/maps"
mkdir -p $MAP_DIR

# 保存地图，添加时间戳
MAP_NAME="mymap"
rosrun map_server map_saver -f $MAP_DIR/$MAP_NAME

echo "地图已保存到 $MAP_DIR/$MAP_NAME.pgm"