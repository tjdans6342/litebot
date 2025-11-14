#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ROS 모드로 LiteBot를 실행하는 예제 코드
"""
import sys
import os

# 프로젝트 루트 경로를 Python path에 추가
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

import rospy
from litebot.bot import LiteBot


def main():
    """ROS 모드로 LiteBot 실행"""
    # ROS 노드 초기화
    rospy.init_node('litebot', anonymous=True)
    
    bot = LiteBot(mode="ros")
    
    rospy.loginfo("LiteBot started")
    
    # ROS 루프 (rospy.is_shutdown() 체크)
    rate = rospy.Rate(20)  # 20 Hz
    
    while not rospy.is_shutdown():
        observations, action = bot.step()
        
        if action:
            cmd, value = action
            rospy.loginfo("Action: {} - {}".format(cmd, value))
        
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
