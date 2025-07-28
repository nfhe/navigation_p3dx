#!/usr/bin/env python

import rospy
import rostopic
import time

def check_topics():
    rospy.init_node('topic_checker', anonymous=True)
    print("等待2秒让ROS节点完全初始化...")
    time.sleep(2)
    
    print("\n获取当前可用的话题列表...")
    topics = rostopic.get_topic_list()
    topic_names = [t[0] for t in topics[0]]
    
    print("\n所有可用话题:")
    for topic in sorted(topic_names):
        print(f" - {topic}")
    
    print("\n搜索特定话题:")
    
    odom_topics = [t for t in topic_names if 'odom' in t.lower()]
    print("\n里程计(odom)相关话题:")
    for t in odom_topics:
        print(f" - {t}")
    
    scan_topics = [t for t in topic_names if 'scan' in t.lower() or 'laser' in t.lower()]
    print("\n激光雷达(scan/laser)相关话题:")
    for t in scan_topics:
        print(f" - {t}")
    
    cmd_vel_topics = [t for t in topic_names if 'cmd_vel' in t.lower() or 'cmdvel' in t.lower()]
    print("\n速度命令(cmd_vel)相关话题:")
    for t in cmd_vel_topics:
        print(f" - {t}")

if __name__ == '__main__':
    try:
        check_topics()
    except rospy.ROSInterruptException:
        pass 