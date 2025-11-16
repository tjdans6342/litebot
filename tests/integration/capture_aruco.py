#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
통합 테스트 - LiteBot으로 주행/관측/트리거를 묶고,
트리거 결과가 'aruco'일 때 현재 프레임을 캡처합니다.
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
    rospy.init_node("litebot_aruco_capture", anonymous=False)
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
            "aruco": litebot.observer.observe_aruco(litebot.images.get("original", litebot.frame)),
            # 다른 감지는 필요 시 확장
        }

        # 4. 트리거 매니저가 적절한 액션과 우세 트리거명을 반환
        action, source = litebot.trigger_manager.step(observations)

        # 5. 액션 실행: 우세 트리거가 아루코면 캡처, 그 외는 정상 실행
        if action:
            cmd, val = action
            if source != 'lane':
                    rospy.loginfo("[LiteBot] source=%s action=%s value=%s", source, action[0], action[1])
            litebot.action_executor.execute(action)
            # if source == "aruco":
            #     ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            #     # 최근 감지된 아루코 ID를 트리거에서 조회해 파일명에 포함
            #     aruco_tr = None
            #     try:
            #         for t in litebot.trigger_manager.triggers:
            #             if t.__class__.__name__ == "ArucoTrigger":
            #                 aruco_tr = t
            #                 break
            #     except Exception:
            #         aruco_tr = None
            #     marker_id = aruco_tr.get_last_id() if aruco_tr and hasattr(aruco_tr, "get_last_id") else None
            #     suffix = "aruco_{}".format(marker_id) if marker_id is not None else "aruco"
            #     path = os.path.join(save_dir, "{}_{}.jpg".format(suffix, ts))
            #     litebot.action_executor.execute(("capture", {"image": litebot.frame, "save_path": path}))
            #     rospy.loginfo("[LiteBot] aruco captured: %s", path)
            # else:
            #     litebot.action_executor.execute(action)
            #     if source != 'lane':
            #         rospy.loginfo("[LiteBot] source=%s action=%s value=%s", source, action[0], action[1])

        rate.sleep()


if __name__ == "__main__":
    main()


