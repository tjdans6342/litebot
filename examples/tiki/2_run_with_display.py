#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Tiki 모드의 LiteBot 실행 예제 (이미지 디스플레이 포함).
# TODO: to include hough image
"""
import os
import sys
import time

# 프로젝트 루트를 Python path에 추가
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from litebot.bot import LiteBot  # noqa: E402
from litebot.utils.display_utils import (
    display_images,
    annotate_lane_info,
    visualize_hough_image
)

IMAGE_TARGETS = [
    # "original",
    "bev",
    "filtered",
    # "gray",
    # "blur",
    "component_binary_img",
    "binary",
    "canny",
    "hough",
]


def main():
    litebot = LiteBot(mode="tiki")
    loop_rate = 20  # Hz
    sleep_duration = 1.0 / loop_rate  # 0.05초

    try:
        while True:
            # 1. 프레임 캡처
            litebot.frame = litebot.camera.get_frame()

            # 프레임이 없으면 처리 중단
            if litebot.frame is None:
                time.sleep(sleep_duration)
                continue

            # 2. 이미지 처리
            litebot.images = litebot.image_processor.get_images(litebot.frame)

            # images가 None이면 빈 딕셔너리로 처리
            if litebot.images is None:
                litebot.images = {}

            # 3. 감지 수행
            observations = {
                "lane": litebot.observer.observe_lines(litebot.images.get("hough")),
                # "aruco": litebot.observer.observe_aruco(litebot.images["original"]),
                # "pothole": litebot.observer.observe_pothole(litebot.images["binary"]),
                # "qr_codes": litebot.observer.observe_qr_codes(litebot.images["original"]),
                # 필요한 경우 다른 감지 추가
            }

            # hough 이미지 시각화 (lane 정보가 있는 경우)
            if litebot.images:
                lane_info = observations.get("lane")
                if lane_info and lane_info.get("exist_lines", True) and "hough" in litebot.images:
                    visualized = visualize_hough_image(litebot.images["hough"], lane_info)
                    litebot.images["hough"] = annotate_lane_info(visualized, lane_info)

            display_images(litebot.images, image_targets=IMAGE_TARGETS)

            # 4. 트리거 매니저들이 적절한 액션과 우세 트리거명을 반환
            # motor 리소스에 대한 관찰만 처리 (lane, aruco, pothole)
            motor_obs = {
                "lane": observations.get("lane"),
                # "aruco": litebot.observer.observe_aruco(litebot.images.get("original")),
                # "pothole": litebot.observer.observe_pothole(litebot.images.get("binary")),
            }
            
            motor_action, motor_source = litebot.trigger_managers["motor"].step(motor_obs)
            
            # 5. 액션 실행
            if motor_action:
                litebot.action_executor.execute(motor_action)
                print("[LiteBot] source={} action={} value={}".format(
                    motor_source, motor_action[0], motor_action[1]))

            time.sleep(sleep_duration)

    except KeyboardInterrupt:
        print("\n[LiteBot] Interrupted by user. Shutting down...")


if __name__ == "__main__":
    main()

