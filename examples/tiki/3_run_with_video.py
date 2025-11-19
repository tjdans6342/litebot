#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
LiteBot 주행 + 비디오 캡처 예제 (Tiki 모드).
"""
import os
import sys
import time

# 프로젝트 루트를 Python path에 추가
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from litebot.bot import LiteBot  # noqa: E402
from litebot.analysis.video_recorder import VideoRecorder  # noqa: E402
from litebot.analysis.analysis_manager import AnalysisManager


def main():
    bot = LiteBot(mode="tiki")
    loop_rate = 20  # Hz
    sleep_duration = 1.0 / loop_rate  # 0.05초

    recorder = VideoRecorder()
    # recorder.set_save_path("recordings/tiki_run.avi")
    recorder.start_recording()
    print("[LiteBot] Video recording started: {}".format(recorder.save_path))

    try:
        while True:
            resource_obs_pairs, actions, sources = bot.step()

            if bot.frame is not None:
                try:
                    manager = AnalysisManager()
                    grid_image = manager.make_images_grid(list(bot.images.values()))
                    recorder.add_frame(grid_image)
                    # recorder.add_frame(bot.images['original'])
                except Exception as exc:
                    print("[LiteBot] failed to add frame to recorder: {}".format(exc))

            if resource_obs_pairs is None:
                time.sleep(sleep_duration)
                continue

            # 모든 리소스별 액션 로그 출력 (lane 제외)
            for resource_type, action in actions.items():
                source = sources.get(resource_type)
                if action and source != 'lane':
                    print("[LiteBot] resource={} action={} value={}".format(
                        resource_type, action[0], action[1]))

            time.sleep(sleep_duration)

    except KeyboardInterrupt:
        print("\n[LiteBot] Interrupted by user. Shutting down...")
    finally:
        recorder.stop_recording()
        print("[LiteBot] Video recording finished.")


if __name__ == "__main__":
    main()

