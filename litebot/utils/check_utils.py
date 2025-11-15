#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    check_utils
    포트홀, 차선 등 다양한 체크 로직을 분리해서 관리하는 모듈
"""

import numpy as np


def check_exist_pothole(binary_image,
                        top_ratio=0.3,
                        top_center_ratio=0.15,
                        side_ratio=0.1,
                        mid_range=(0.5, 0.7),
                        white_ratio_thresh=0.1):
    """
        간단한 형태 기반 조건으로 포트홀 존재 여부를 판별합니다.
    """
    if binary_image is None:
        return False

    h, w = binary_image.shape[:2]

    top_side_area1 = binary_image[:int(h * top_ratio), :int(w * side_ratio)]
    top_side_area2 = binary_image[:int(h * top_ratio), int(w * (1 - side_ratio)):]
    top_center_area = binary_image[:int(h * top_center_ratio), int(0.2 * w):int(0.8 * w)]
    mid_area = binary_image[int(h * mid_range[0]):int(h * mid_range[1]), :]

    total = top_center_area.size or 1
    white_per = float(np.sum(top_center_area == 255)) / float(total)

    return (
        white_per > white_ratio_thresh
        and mid_area.max() == 0
        and top_side_area1.max() == 0
        and top_side_area2.max() == 0
    )