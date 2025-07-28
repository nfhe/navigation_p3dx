#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

def send_goal(x, y):
    rospy.init_node('send_goal_node')
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    dist_pub = rospy.Publisher('/dist_to_goal_th', Float32, queue_size=1)
    
    # 等待连接
    rospy.sleep(2.0)
    
    # 设置目标点
    goal = PoseStamped()
    goal.header.frame_id = "odom"  # 使用里程计坐标系
    goal.header.stamp = rospy.Time.now()

    # 目标位置 (x, y, z)
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.0

    # 目标方向 (使用四元数表示)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]

    # 设置到达目标点的距离阈值
    dist_th = Float32()
    dist_th.data = 0.5  # 0.5米

    rospy.loginfo("发送导航目标点: x=%f, y=%f", goal.pose.position.x, goal.pose.position.y)
    rospy.loginfo("参考坐标系: %s", goal.header.frame_id)

    # 发布目标点和距离阈值
    goal_pub.publish(goal)
    dist_pub.publish(dist_th)

    rospy.loginfo("请在RViz中查看导航过程")
    rospy.spin()

if __name__ == '__main__':
    try:
        # 在这里设置目标点坐标
        target_x = 38.0
        target_y = 0.0
        
        # 调用函数发送目标点
        send_goal(target_x, target_y)
    except rospy.ROSInterruptException:
        pass