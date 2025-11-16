#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
통합 테스트 - LiteBot으로 주행/관측/트리거를 묶고,
트리거 결과가 'pothole'일 때 현재 프레임을 캡처합니다.
"""
from __future__ import print_function

import os
from datetime import datetime

import rospy

from litebot.bot import LiteBot


def _ensure_out_dir():
    out_dir = os.path.join("tests", "_out")
    try:
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)
    except Exception:
        pass
    return out_dir


def main():
    rospy.init_node("litebot_pothole_capture", anonymous=False)
    litebot = LiteBot(mode="ros")
    rate = rospy.Rate(20)

    save_dir = _ensure_out_dir()

    while not rospy.is_shutdown():
        # 1. 프레임 캡처
        litebot.frame = litebot.camera.get_frame()

        # 프레임이 없으면 처리 중단
        if litebot.frame is None:
            rate.sleep()
            continue

        # 2. 이미지 처리
        litebot.images = litebot.image_processor.get_images(litebot.frame)
        if litebot.images is None:
            litebot.images = {}

        # 3. 감지 수행 (필요 항목만 추가)
        observations = {
            "lane": litebot.observer.observe_lines(litebot.images.get("hough", litebot.frame)),
            "pothole": litebot.observer.observe_pothole(litebot.images.get("binary", litebot.frame)),
            # 예: pothole 감지를 별도 추가하고 싶으면 아래처럼 확장
        }

        # 4. 트리거 매니저가 적절한 액션과 우세 트리거명을 반환
        action, source = litebot.trigger_manager.step(observations)

        # 5. 액션 실행: 우세 트리거가 포트홀이면 캡처, 그 외는 정상 실행
        if action:
            cmd, val = action
            litebot.action_executor.execute(action)
            if source != 'lane':
                rospy.loginfo("[LiteBot] source=%s action=%s value=%s", source, action[0], action[1])
            # if source == "pothole":
            #     ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            #     path = os.path.join(save_dir, "pothole_{}.jpg".format(ts))
            #     litebot.action_executor.execute(("capture", {"image": litebot.frame, "save_path": path}))
            #     rospy.loginfo("[LiteBot] pothole captured: %s", path)
            # else:
            #     litebot.action_executor.execute(action)
            #     if source != 'lane':
            #         rospy.loginfo("[LiteBot] source=%s action=%s value=%s", source, action[0], action[1])

        rate.sleep()


if __name__ == "__main__":
    main()


