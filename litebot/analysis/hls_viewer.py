#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
HLS/RGB 범위 분석 및 시각화를 위한 인터랙티브 도구

기능:
- 비디오 파일을 재생하면서 H/L/S 및 R/G/B 최소·최대 범위를 트랙바로 조절
- HLS와 RGB 범위를 모두 만족하는 픽셀만 보존하고 나머지는 검게 마스킹
- 남은 픽셀들의 H, L, S 및 R, G, B 평균값을 화면에 출력
- 재생 위치를 트랙바(Position)로 직접 이동

사용법:
    python -m litebot.analysis.hls_viewer --video ./videos/sample.mp4

조작:
- 트랙바: H/L/S 및 R/G/B 최솟값·최댓값 조절, Position으로 재생 위치 이동
- Space: 일시정지/재생
- R: 비디오 처음으로 되감기
- Q 또는 ESC: 종료

주의:
- HLS와 RGB 마스크를 AND 연산으로 결합하므로, 두 조건을 모두 만족하는 픽셀만 선택됩니다.
- 검은색 도로 필터링 시: L 값 낮게 + R/G/B 값 모두 낮게 설정하면 더 정확합니다.
"""

import argparse
import os

import cv2
import numpy as np


WINDOW_NAME = "LiteBot HLS/RGB Viewer"
POSITION_TRACKBAR = "Position"
TRACKBAR_CONFIG = [
    # HLS 트랙바
    ("H_min", 0, 180, 0),
    ("H_max", 0, 180, 180),
    ("L_min", 0, 255, 0),
    ("L_max", 0, 255, 255),
    ("S_min", 0, 255, 0),
    ("S_max", 0, 255, 255),
    # RGB 트랙바
    ("R_min", 0, 255, 0),
    ("R_max", 0, 255, 255),
    ("G_min", 0, 255, 0),
    ("G_max", 0, 255, 255),
    ("B_min", 0, 255, 0),
    ("B_max", 0, 255, 255),
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


def _create_trackbars(total_frames, window_width=None, window_height=None):
    """
    트랙바를 생성하고 창 크기를 설정
    
    Args:
        total_frames: 비디오의 전체 프레임 수 (이미지인 경우 0)
        window_width: 창 너비 (None이면 자동)
        window_height: 창 높이 (None이면 자동)
    """
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    
    # 창 크기를 먼저 설정 (트랙바 생성 전에 크기를 지정하면 더 잘 작동함)
    if window_width is not None and window_height is not None:
        cv2.resizeWindow(WINDOW_NAME, window_width, window_height)
    
    for name, _, max_val, initial in TRACKBAR_CONFIG:
        cv2.createTrackbar(name, WINDOW_NAME, initial, max_val, _nothing)

    if total_frames > 0:
        max_pos = max(0, total_frames - 1)
        cv2.createTrackbar(POSITION_TRACKBAR, WINDOW_NAME, 0, max_pos, _on_position_change)
        TRACKBAR_STATE["position_enabled"] = True
    else:
        TRACKBAR_STATE["position_enabled"] = False


def _get_window_size_with_trackbars(image_width, image_height):
    """
    트랙바를 고려한 창 크기 계산
    Windows에서 트랙바가 있는 창의 크기 조절이 제대로 작동하도록
    트랙바 높이를 고려하여 전체 창 크기를 계산
    """
    # 트랙바 높이 추정 (각 트랙바 약 20-25px, 여백 포함)
    num_trackbars = len(TRACKBAR_CONFIG)
    if TRACKBAR_STATE.get("position_enabled", False):
        num_trackbars += 1
    trackbar_height = num_trackbars * 25 + 50  # 트랙바 높이 + 여백
    
    # 정보 바 높이 (약 40px)
    info_bar_height = 40
    
    # 전체 창 크기
    total_width = image_width
    total_height = image_height + trackbar_height + info_bar_height
    
    return total_width, total_height




def _window_visible():
    try:
        prop = cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE)
        return prop >= 1
    except cv2.error:
        return False


def _get_range_from_trackbars():
    # HLS 범위
    h_min = cv2.getTrackbarPos("H_min", WINDOW_NAME)
    h_max = cv2.getTrackbarPos("H_max", WINDOW_NAME)
    l_min = cv2.getTrackbarPos("L_min", WINDOW_NAME)
    l_max = cv2.getTrackbarPos("L_max", WINDOW_NAME)
    s_min = cv2.getTrackbarPos("S_min", WINDOW_NAME)
    s_max = cv2.getTrackbarPos("S_max", WINDOW_NAME)

    hls_lower = np.array([min(h_min, h_max), min(l_min, l_max), min(s_min, s_max)], dtype=np.uint8)
    hls_upper = np.array([max(h_min, h_max), max(l_min, l_max), max(s_min, s_max)], dtype=np.uint8)
    
    # RGB 범위
    r_min = cv2.getTrackbarPos("R_min", WINDOW_NAME)
    r_max = cv2.getTrackbarPos("R_max", WINDOW_NAME)
    g_min = cv2.getTrackbarPos("G_min", WINDOW_NAME)
    g_max = cv2.getTrackbarPos("G_max", WINDOW_NAME)
    b_min = cv2.getTrackbarPos("B_min", WINDOW_NAME)
    b_max = cv2.getTrackbarPos("B_max", WINDOW_NAME)
    
    rgb_lower = np.array([min(b_min, b_max), min(g_min, g_max), min(r_min, r_max)], dtype=np.uint8)  # BGR 순서
    rgb_upper = np.array([max(b_min, b_max), max(g_min, g_max), max(r_min, r_max)], dtype=np.uint8)  # BGR 순서
    
    return hls_lower, hls_upper, rgb_lower, rgb_upper


def _format_mean_text(hls_mean, rgb_mean, pixel_ratio):
    h, l, s = hls_mean
    b, g, r = rgb_mean  # BGR 순서
    return "H:{:.1f} L:{:.1f} S:{:.1f} | R:{:.1f} G:{:.1f} B:{:.1f} | Pixels:{:.1f}%".format(
        h, l, s, r, g, b, pixel_ratio*100
    )


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
    parser.add_argument("--video", required=True, help="분석할 비디오 또는 이미지 파일 경로")
    parser.add_argument("--loop", action="store_true", help="영상 끝에 도달하면 자동으로 처음부터 반복 (비디오만)")
    parser.add_argument("--scale", type=float, default=1.0, help="표시 배율 (1.0은 원본)")
    parser.add_argument("--fps", type=float, default=30.0, help="재생 프레임 속도 (FPS, 0이면 최소 지연, 비디오만)")
    parser.add_argument("--skip-errors", action="store_true", help="손상된 프레임을 건너뜀 (비디오만)")
    return parser.parse_args()


def _is_image_file(filepath):
    """파일이 이미지인지 확인"""
    image_extensions = ('.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.tif', '.webp')
    return filepath.lower().endswith(image_extensions)


def main():
    args = parse_args()
    input_path = args.video

    if not os.path.exists(input_path):
        raise FileNotFoundError("File not found: {}".format(input_path))

    is_image = _is_image_file(input_path)
    
    if is_image:
        # 이미지 파일 처리
        frame = cv2.imread(input_path)
        if frame is None:
            raise IOError("Failed to load image: {}".format(input_path))
        
        _create_trackbars(0)  # 이미지는 Position 트랙바 불필요
        current_frame = frame.copy()
        need_resize = True
        cap = None
        total_frames = 0
        paused = False
        delay_ms = 1
    else:
        # 비디오 파일 처리
        cap = cv2.VideoCapture(input_path)
        if not cap.isOpened():
            raise IOError("Failed to open video: {}".format(input_path))

        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        _create_trackbars(total_frames)
        paused = False
        current_frame = None
        need_resize = True
        delay_ms = 1 if args.fps <= 0 else max(1, int(1000.0 / args.fps))

    while True:
        if not _window_visible():
            break

        if is_image:
            # 이미지인 경우 항상 같은 프레임 사용
            frame = current_frame
        else:
            # 비디오인 경우
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

        hls_lower, hls_upper, rgb_lower, rgb_upper = _get_range_from_trackbars()
        
        # HLS 마스크 생성
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        hls_mask = cv2.inRange(hls, hls_lower, hls_upper)
        
        # RGB 마스크 생성 (BGR 순서로 변환 필요)
        rgb_mask = cv2.inRange(frame, rgb_lower, rgb_upper)
        
        # HLS 마스크와 RGB 마스크를 AND 연산으로 결합
        mask = cv2.bitwise_and(hls_mask, rgb_mask)
        
        masked = cv2.bitwise_and(frame, frame, mask=mask)

        total_pixels = mask.size
        masked_pixels = cv2.countNonZero(mask)

        if masked_pixels > 0:
            hls_mean = cv2.mean(hls, mask=mask)[:3]
            rgb_mean = cv2.mean(frame, mask=mask)[:3]  # BGR 순서
        else:
            hls_mean = (0.0, 0.0, 0.0)
            rgb_mean = (0.0, 0.0, 0.0)

        ratio = masked_pixels / float(total_pixels)
        combined = np.hstack((frame, masked))
        text = _format_mean_text(hls_mean, rgb_mean, ratio)
        _draw_overlay(combined, text)

        display_frame = combined
        if args.scale != 1.0 and args.scale > 0:
            width = max(1, int(combined.shape[1] * args.scale))
            height = max(1, int(combined.shape[0] * args.scale))
            display_frame = cv2.resize(combined, (width, height), interpolation=cv2.INTER_NEAREST)

        cv2.imshow(WINDOW_NAME, display_frame)

        if not is_image and TRACKBAR_STATE["position_enabled"]:
            current_pos = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
            TRACKBAR_STATE["updating"] = True
            cv2.setTrackbarPos(
                POSITION_TRACKBAR,
                WINDOW_NAME,
                min(current_pos, max(0, total_frames - 1)),
            )
            TRACKBAR_STATE["updating"] = False

        # 창 크기를 조절 (scale이 적용된 경우에만, 한 번만)
        # Windows에서 트랙바가 있는 창의 가로 크기 조절이 제대로 작동하지 않는 제한사항이 있음
        # 이를 해결하기 위해 창을 재생성하는 방법 사용 (한 번만)
        if args.scale != 1.0 and need_resize:
            # scale이 적용된 경우에만 창 크기 조절 시도 (한 번만)
            # Windows의 제한으로 인해 완전한 해결은 어려울 수 있음
            try:
                # 트랙바를 고려한 전체 창 크기 계산
                window_width, window_height = _get_window_size_with_trackbars(
                    display_frame.shape[1], display_frame.shape[0]
                )
                # 현재 트랙바 값들을 저장
                trackbar_values = {}
                for name, _, _, _ in TRACKBAR_CONFIG:
                    try:
                        trackbar_values[name] = cv2.getTrackbarPos(name, WINDOW_NAME)
                    except:
                        pass
                
                position_value = None
                if TRACKBAR_STATE.get("position_enabled", False):
                    try:
                        position_value = cv2.getTrackbarPos(POSITION_TRACKBAR, WINDOW_NAME)
                    except:
                        pass
                
                # 창을 닫고 다시 생성 (크기 조절을 위해)
                cv2.destroyWindow(WINDOW_NAME)
                _create_trackbars(
                    total_frames if not is_image else 0,
                    window_width,
                    window_height
                )
                
                # 트랙바 값 복원
                for name, _, max_val, _ in TRACKBAR_CONFIG:
                    if name in trackbar_values:
                        cv2.setTrackbarPos(name, WINDOW_NAME, trackbar_values[name])
                
                if TRACKBAR_STATE.get("position_enabled", False) and position_value is not None:
                    cv2.setTrackbarPos(POSITION_TRACKBAR, WINDOW_NAME, position_value)
                
                need_resize = False  # 한 번만 재생성하도록 플래그 설정
            except:
                pass
        elif need_resize:
            try:
                # 트랙바를 고려한 전체 창 크기 계산
                window_width, window_height = _get_window_size_with_trackbars(
                    display_frame.shape[1], display_frame.shape[0]
                )
                cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(WINDOW_NAME, window_width, window_height)
            except:
                pass
            need_resize = False

        key = cv2.waitKey(delay_ms) & 0xFF

        if key in (ord("q"), 27):
            break
        elif key == ord(" ") and not is_image:
            paused = not paused
        elif key == ord("r") and not is_image:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    if not is_image and cap is not None:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

"""
Example:
    # 비디오 파일 사용 예시:
    # "--loop" 옵션을 주면 동영상을 끝까지 본 뒤 다시 처음부터 반복 재생합니다.
    python -m litebot.analysis.hls_viewer --video recordings/example_video.mp4 --loop

    # "--scale" 옵션을 주면 화면 크기를 조절합니다.
    python -m litebot.analysis.hls_viewer --video recordings/example_video.mp4 --scale 0.5

    # "--skip-errors" 를 주면 깨진 프레임을 건너뜁니다.
    python -m litebot.analysis.hls_viewer --video recordings/example_video.mp4 --skip-errors

    # 여러 옵션을 조합해서 사용할 수 있습니다.
    python -m litebot.analysis.hls_viewer --video recordings/example_video.mp4 --loop --scale 0.8 --skip-errors
    
    # "--fps" 옵션으로 영상 표시 속도를 임의로 조절할 수 있습니다 (예: 10fps로 느리게 재생).
    python -m litebot.analysis.hls_viewer --video recordings/example_video.mp4 --fps 10

    # 이미지 파일 사용 예시:
    # 이미지 파일도 분석할 수 있습니다 (jpg, png, bmp 등 지원).
    python -m litebot.analysis.hls_viewer --video recordings/example_image.jpg

    # 이미지 파일도 "--scale" 옵션으로 크기 조절 가능합니다.
    python -m litebot.analysis.hls_viewer --video recordings/example_image.jpg --scale 0.5

    # PNG 이미지 파일 분석 예시.
    python -m litebot.analysis.hls_viewer --video recordings/2024_maicon_photo.jpg
"""
