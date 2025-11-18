#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS 모드의 가장 단순한 LiteBot 실행 예제.
"""
import os
import sys

import rospy

# 프로젝트 루트를 Python path에 추가
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from litebot.bot import LiteBot  # noqa: E402


def main():
    rospy.init_node("litebot_simple_runner", anonymous=False)
    bot = LiteBot(mode="ros")
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        resource_obs_pairs, actions, sources = bot.step()

        if resource_obs_pairs is None:
            rate.sleep()
            continue

        # 모든 리소스별 액션 로그 출력
        for resource_type, action in actions.items():
            if action:
                rospy.loginfo("[LiteBot] resource=%s action=%s value=%s", resource_type, action[0], action[1])

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

