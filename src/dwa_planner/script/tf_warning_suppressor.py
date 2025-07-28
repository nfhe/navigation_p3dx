#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import rospy
import sys
import roslib
import subprocess

# TF警告过滤器节点
class TFWarningSuppressor:
    def __init__(self):
        rospy.init_node('tf_warning_suppressor', anonymous=True)
        rospy.loginfo("TF警告抑制器已启动")
        
        # 设置ROS日志级别
        os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = ''  # 禁用默认配置
        os.environ['ROSCONSOLE_FORMAT'] = '[${severity}] ${message}'
        
        # 设置过滤模式
        self.tf_patterns = [
            re.compile(r'.*TF_REPEATED_DATA.*'),
            re.compile(r'.*transform from .* to .* was already registered.*'),
            re.compile(r'.*ignoring data with redundant timestamp.*')
        ]
        
        # 重定向stderr
        self.old_stderr = sys.stderr
        sys.stderr = self
        
        rospy.loginfo("TF警告过滤器已激活")
        
    def write(self, data):
        # 过滤TF警告
        if any(pattern.match(data) for pattern in self.tf_patterns):
            return
        
        # 写入其他消息
        self.old_stderr.write(data)
        
    def flush(self):
        self.old_stderr.flush()
        
    def run(self):
        # 保持节点运行
        rospy.spin()
        
if __name__ == '__main__':
    try:
        suppressor = TFWarningSuppressor()
        suppressor.run()
    except rospy.ROSInterruptException:
        pass 