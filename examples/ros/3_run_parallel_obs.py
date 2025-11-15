#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
LiteBot 관찰 결과를 두 개의 작업으로 병렬 처리하는 예제.
"""
import os
import sys
from concurrent.futures import ThreadPoolExecutor

import rospy

# 프로젝트 루트를 Python path에 추가
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from litebot.bot import LiteBot  # noqa: E402


def process_lane(obs):
    lane = obs.get("lane")
    if not lane:
        return "lane", "no lane info"
    status = "lines_detected" if lane.get("exist_lines", True) else "lost"
    offset = lane.get("offset", 0.0)
    return "lane", "status={} offset={:.3f}".format(status, offset)


def process_events(obs):
    pothole = obs.get("pothole")
    aruco = obs.get("aruco")
    messages = []
    if pothole:
        messages.append("pothole detected")
    if aruco:
        messages.append("aruco={} markers".format(len(aruco.get("markers", []))))
    if not messages:
        messages.append("no events")
    return "events", ", ".join(messages)


def main():
    rospy.init_node("litebot_parallel_runner", anonymous=False)
    bot = LiteBot(mode="ros")
    rate = rospy.Rate(20)
    executor = ThreadPoolExecutor(max_workers=2)

    while not rospy.is_shutdown():
        observations, action = bot.step()

        if observations is None:
            rate.sleep()
            continue

        futures = [
            executor.submit(process_lane, observations),
            executor.submit(process_events, observations),
        ]
        for future in futures:
            category, message = future.result()
            rospy.loginfo("[Parallel %s] %s", category, message)

        if action:
            rospy.loginfo("[LiteBot] action=%s value=%s", action[0], action[1])

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

