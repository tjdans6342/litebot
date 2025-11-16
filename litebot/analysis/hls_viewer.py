#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
HLS 범위 분석 및 시각화를 위한 인터랙티브 도구

기능:
- 비디오 파일을 재생하면서 H/L/S 최소·최대 범위를 트랙바로 조절
- 선택된 범위에 해당하는 픽셀만 보존하고 나머지는 검게 마스킹
- 남은 픽셀들의 H, L, S 평균값을 화면에 출력
- 재생 위치를 트랙바(Position)로 직접 이동

사용법:
    python -m litebot.analysis.hls_viewer --video ./videos/sample.mp4

조작:
- 트랙바: H/L/S 최솟값·최댓값 조절, Position으로 재생 위치 이동
- Space: 일시정지/재생
- R: 비디오 처음으로 되감기
- Q 또는 ESC: 종료
"""

import argparse
import os

import cv2
import numpy as np


WINDOW_NAME = "LiteBot HLS Viewer"
POSITION_TRACKBAR = "Position"
TRACKBAR_CONFIG = [
    ("H_min", 0, 180, 0),
    ("H_max", 0, 180, 180),
    ("L_min", 0, 255, 0),
    ("L_max", 0, 255, 255),
    ("S_min", 0, 255, 0),
    ("S_max", 0, 255, 255),
]
TRACKBAR_STATE = {
    "updating": False,
    "seek_request": None,
    "position_enabled": False,
}


def _nothing(_):
    pass


def _on_position_change(pos):
    if TRACKBAR_STATE.get("updating"):
        return
    TRACKBAR_STATE["seek_request"] = pos


def _create_trackbars(total_frames):
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    for name, _, max_val, initial in TRACKBAR_CONFIG:
        cv2.createTrackbar(name, WINDOW_NAME, initial, max_val, _nothing)

    if total_frames > 0:
        max_pos = max(0, total_frames - 1)
        cv2.createTrackbar(POSITION_TRACKBAR, WINDOW_NAME, 0, max_pos, _on_position_change)
        TRACKBAR_STATE["position_enabled"] = True
    else:
        TRACKBAR_STATE["position_enabled"] = False


def _window_visible():
    try:
        prop = cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE)
        return prop >= 1
    except cv2.error:
        return False


def _get_range_from_trackbars():
    h_min = cv2.getTrackbarPos("H_min", WINDOW_NAME)
    h_max = cv2.getTrackbarPos("H_max", WINDOW_NAME)
    l_min = cv2.getTrackbarPos("L_min", WINDOW_NAME)
    l_max = cv2.getTrackbarPos("L_max", WINDOW_NAME)
    s_min = cv2.getTrackbarPos("S_min", WINDOW_NAME)
    s_max = cv2.getTrackbarPos("S_max", WINDOW_NAME)

    lower = np.array([min(h_min, h_max), min(l_min, l_max), min(s_min, s_max)], dtype=np.uint8)
    upper = np.array([max(h_min, h_max), max(l_min, l_max), max(s_min, s_max)], dtype=np.uint8)
    return lower, upper


def _format_mean_text(mean_vals, pixel_ratio):
    h, l, s = mean_vals
    return "H:{:.1f}  L:{:.1f}  S:{:.1f}  Pixels:{:.1f}%".format(h, l, s, pixel_ratio*100)


def _draw_overlay(image, text):
    cv2.rectangle(image, (0, 0), (image.shape[1], 40), (0, 0, 0), thickness=-1)
    cv2.putText(
        image,
        text,
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        "[Space]Pause  [R]Restart  [Q/ESC]Quit",
        (10, image.shape[0] - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )


def parse_args():
    parser = argparse.ArgumentParser(description="LiteBot HLS 범위 분석 도구")
    parser.add_argument("--video", required=True, help="분석할 비디오 파일 경로")
    parser.add_argument("--loop", action="store_true", help="영상 끝에 도달하면 자동으로 처음부터 반복")
    parser.add_argument("--scale", type=float, default=1.0, help="표시 배율 (1.0은 원본)")
    parser.add_argument("--fps", type=float, default=30.0, help="재생 프레임 속도 (FPS, 0이면 최소 지연)")
    parser.add_argument("--skip-errors", action="store_true", help="손상된 프레임을 건너뜀")
    return parser.parse_args()


def main():
    args = parse_args()
    video_path = args.video

    if not os.path.exists(video_path):
        raise FileNotFoundError("Video not found: {}".format(video_path))

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise IOError("Failed to open video: {}".format(video_path))

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    _create_trackbars(total_frames)
    paused = False
    current_frame = None
    need_resize = True
    delay_ms = 1 if args.fps <= 0 else max(1, int(1000.0 / args.fps))

    while True:
        if not _window_visible():
            break

        if TRACKBAR_STATE["position_enabled"] and TRACKBAR_STATE["seek_request"] is not None:
            target = TRACKBAR_STATE["seek_request"]
            TRACKBAR_STATE["seek_request"] = None
            if total_frames > 0:
                target = max(0, min(total_frames - 1, target))
            cap.set(cv2.CAP_PROP_POS_FRAMES, target)
            current_frame = None

        if not paused or current_frame is None:
            ret, frame = cap.read()
            if not ret:
                if args.loop:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                break
            try:
                current_frame = frame.copy()
            except cv2.error:
                if args.skip_errors:
                    continue
                raise
        frame = current_frame

        lower, upper = _get_range_from_trackbars()
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        mask = cv2.inRange(hls, lower, upper)
        masked = cv2.bitwise_and(frame, frame, mask=mask)

        total_pixels = mask.size
        masked_pixels = cv2.countNonZero(mask)

        if masked_pixels > 0:
            mean_vals = cv2.mean(hls, mask=mask)[:3]
        else:
            mean_vals = (0.0, 0.0, 0.0)

        ratio = masked_pixels / float(total_pixels)
        combined = np.hstack((frame, masked))
        text = _format_mean_text(mean_vals, ratio)
        _draw_overlay(combined, text)

        display_frame = combined
        if args.scale != 1.0 and args.scale > 0:
            width = max(1, int(combined.shape[1] * args.scale))
            height = max(1, int(combined.shape[0] * args.scale))
            display_frame = cv2.resize(combined, (width, height), interpolation=cv2.INTER_NEAREST)

        cv2.imshow(WINDOW_NAME, display_frame)
        if need_resize or args.scale == 1.0:
            cv2.resizeWindow(WINDOW_NAME, display_frame.shape[1], display_frame.shape[0])
            need_resize = False

        if TRACKBAR_STATE["position_enabled"]:
            current_pos = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
            TRACKBAR_STATE["updating"] = True
            cv2.setTrackbarPos(
                POSITION_TRACKBAR,
                WINDOW_NAME,
                min(current_pos, max(0, total_frames - 1)),
            )
            TRACKBAR_STATE["updating"] = False

        key = cv2.waitKey(delay_ms) & 0xFF

        if key in (ord("q"), 27):
            break
        elif key == ord(" "):
            paused = not paused
        elif key == ord("r"):
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

"""
Example:
    # "--loop" 옵션을 주면 동영상을 끝까지 본 뒤 다시 처음부터 반복 재생합니다.
    python -m litebot.analysis.hls_viewer --video recordings/example_video.mp4 --loop
    # "--scale" 옵션을 주면 화면 크기를 조절합니다.
    python -m litebot.analysis.hls_viewer --video recordings/example_video.mp4 --scale 0.5
    # "--skip-errors" 를 주면 깨진 프레임을 건너뜁니다.
    python -m litebot.analysis.hls_viewer --video recordings/example_video.mp4 --skip-errors
    # 여러 옵션을 조합해서 사용할 수 있습니다.
    python -m litebot.analysis.hls_viewer --video recordings/example_video.mp4 --loop --scale 0.8 --skip-errors
    # 특정 구간만 재생하고 싶다면 --start-frame, --end-frame 옵션도 사용할 수 있습니다.
    python -m litebot.analysis.hls_viewer --video recordings/example_video.mp4 --start-frame 100 --end-frame 200
    # 채널별(H, L, S) 슬라이더 없이 프레임만 보고 싶으면 --no-trackbars 옵션을 쓸 수 있습니다.
    python -m litebot.analysis.hls_viewer --video recordings/example_video.mp4 --no-trackbars
    # "--fps" 옵션으로 영상 표시 속도를 임의로 조절할 수 있습니다 (예: 10fps로 느리게 재생).
    python -m litebot.analysis.hls_viewer --video recordings/example_video.mp4 --fps 10
"""
