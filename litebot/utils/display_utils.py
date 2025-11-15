#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    이미지 표시 유틸리티 함수 모음
"""
import cv2
import numpy as np

DEFAULT_IMAGE_TARGETS = [
    ("Original", "original"),
    ("BEV", "bev"),
    ("Filtered", "filtered"),
    ("gray", "gray"),
    ("Blurred", "blur"),
    ("binary", "binary"),
    ("Canny", "canny"),
    ("Hough", "hough"),
]

DEFAULT_WINDOW_POSITIONS = [
    (0, 0), (600, 0), (1200, 0),
    (0, 600), (600, 600), (1200, 600)
]


def display_images(images, image_targets=None, window_positions=None):
    """
        지정된 이미지들을 OpenCV 창으로 표시합니다.

        Args:
            images (dict): 이미지 딕셔너리 (ImageProcessor 결과 등)
            image_targets (list): (창 이름, 딕셔너리 키) 튜플 리스트
            window_positions (list): 창 위치 리스트 [(x, y), ...]
    """
    if images is None:
        return

    if image_targets is None:
        image_targets = DEFAULT_IMAGE_TARGETS

    for idx, (window_name, key) in enumerate(image_targets):
        if key not in images or images[key] is None:
            continue

        img = images[key]
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        cv2.namedWindow(window_name)
        positions = window_positions or DEFAULT_WINDOW_POSITIONS
        pos = positions[idx % len(positions)]
        cv2.moveWindow(window_name, pos[0], pos[1])
        cv2.imshow(window_name, img)

    cv2.waitKey(1)


def annotate_lane_info(image, lane_info):
    """
        Hough 이미지를 복사해 heading / position 정보를 오버레이한 이미지를 반환합니다.
    """
    if image is None or lane_info is None:
        return image

    annotated = image.copy()
    if len(annotated.shape) == 2:
        annotated = cv2.cvtColor(annotated, cv2.COLOR_GRAY2BGR)

    heading = lane_info.get("heading")
    position = lane_info.get("position")
    texts = []
    if heading is not None:
        texts.append("heading: {:.3f} rad".format(heading))
    if position is not None:
        texts.append("position: {:.3f}".format(position))

    for idx, text in enumerate(texts):
        cv2.putText(
            annotated,
            text,
            (10, 30 + idx * 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )
    return annotated


def visualize_hough_image(hough_img, lane_info):
    """
        슬라이딩 윈도우 결과를 기반으로 Hough 이미지를 시각화합니다.
    """
    if hough_img is None:
        return None
    if lane_info is None:
        return hough_img

    x_points = lane_info.get("x")
    y_points = lane_info.get("y")
    fit = lane_info.get("fit")
    mid_avg = lane_info.get("mid_avg")
    window_width = lane_info.get("window_width")
    nwindows = lane_info.get("nwindows")

    if x_points is None or y_points is None or fit is None:
        return hough_img

    vis = hough_img.copy()
    if len(vis.shape) == 2:
        vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

    h, w = hough_img.shape[:2]
    if window_width is None or window_width <= 0:
        window_width = int(w * 0.1)
    margin = int(window_width // 2)

    if nwindows is None or nwindows <= 0:
        nwindows = 9
    window_height = max(1, int(h / float(nwindows)))

    for cx, cy in zip(x_points, y_points):
        win_yl = int(cy - window_height / 2.0)
        win_yh = int(cy + window_height / 2.0)
        win_xl = int(cx - margin)
        win_xh = int(cx + margin)
        cv2.rectangle(vis, (win_xl, win_yl), (win_xh, win_yh), (0, 255, 0), 2)

    for cx, cy in zip(x_points, y_points):
        cv2.circle(vis, (int(cx), int(cy)), 6, (255, 0, 0), -1)

    if isinstance(fit, (list, tuple, np.ndarray)) and len(fit) == 3:
        y_plot = np.linspace(0, h - 1, h)
        x_fit = fit[0] * y_plot ** 2 + fit[1] * y_plot + fit[2]
        for i in range(1, len(y_plot)):
            cv2.line(
                vis,
                (int(x_fit[i - 1]), int(y_plot[i - 1])),
                (int(x_fit[i]), int(y_plot[i])),
                (0, 255, 255),
                3
            )

    if mid_avg is not None:
        cv2.line(
            vis,
            (int(mid_avg), 0),
            (int(mid_avg), h),
            (255, 100, 255),
            2
        )

    return vis

