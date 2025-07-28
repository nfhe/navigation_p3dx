#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import time

if __name__ == '__main__':
    rospy.init_node('direct_goal_sender')

    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    # 获取ROS参数，如果未指定则使用默认值
    target_frame = rospy.get_param('~target_frame', 'odom')
    
    # 等待publisher初始化
    rospy.sleep(5.0)  # 给系统更多时间初始化
    
    # 创建目标点消息
    goal = PoseStamped()
    goal.header.frame_id = target_frame  # 使用指定的坐标系
    
    goal.pose.position.x = 3.0
    goal.pose.position.y = 0.0
    goal.pose.position.z = 0.0
    
    goal.pose.orientation.w = 1.0  # 默认朝向
    
    rate = rospy.Rate(0.2)  # 5秒发送一次目标点
    
    counter = 0
    
    try:
        while not rospy.is_shutdown():
            # 更新时间戳
            goal.header.stamp = rospy.Time.now()
            
            # 每10次切换目标点
            if counter % 10 == 0:
                goal.pose.position.x = 3.0
                goal.pose.position.y = 0.0
                rospy.loginfo("发布目标点到 /move_base_simple/goal: (3.0, 0.0)")
            elif counter % 10 == 5:
                goal.pose.position.x = 0.0
                goal.pose.position.y = 3.0
                rospy.loginfo("发布目标点到 /move_base_simple/goal: (0.0, 3.0)")
            
            # 发布目标点
            goal_pub.publish(goal)
            
            counter += 1
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
