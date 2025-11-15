#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    주행 영상/이미지 분석을 위한 매니저
"""
import os

import cv2
import numpy as np

from litebot.analysis.video_recorder import VideoRecorder


class AnalysisManager:
    """
        녹화 및 분석 관련 헬퍼 객체를 통합 관리하는 클래스
    """

    def __init__(self):
        """
            AnalysisManager 초기화
        """
        self.video_recorder = VideoRecorder()

    def convert_video_to_images(self, video_path, output_dir, prefix="frame", image_ext=".png"):
        """
            비디오 파일을 프레임 단위 이미지로 추출합니다.

            Args:
                video_path (str): 입력 비디오 경로
                output_dir (str): 프레임 이미지를 저장할 디렉터리

            Returns:
                list[str]: 저장된 이미지 경로 리스트
        """
        if not os.path.exists(video_path):
            raise FileNotFoundError("Video not found: {}".format(video_path))

        os.makedirs(output_dir, exist_ok=True)
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            raise IOError("Failed to open video: {}".format(video_path))

        saved_paths = []
        frame_idx = 0
        success, frame = cap.read()
        while success:
            filename = f"{prefix}_{frame_idx:05d}{image_ext}"
            save_path = os.path.join(output_dir, filename)
            cv2.imwrite(save_path, frame)
            saved_paths.append(save_path)
            frame_idx += 1
            success, frame = cap.read()

        cap.release()
        return saved_paths

    def make_images_grid(self, images, grid_shape=(3, 3), tile_size=(320, 240), fill_color=(0, 0, 0)):
        """
            다수의 이미지를 3x3 형태의 한 장짜리 그리드 이미지로 합칩니다.

            Args:
                images (list[numpy.ndarray]):
                    최대 9장의 이미지 리스트.
                grid_shape (tuple[int, int]):
                    (행, 열) 형태의 그리드 크기.
                tile_size (tuple[int, int]):
                    각 셀에 들어갈 (width, height) 크기.
                fill_color (tuple[int, int, int]):
                    부족한 셀을 채울 BGR 색상.

            Returns:
                numpy.ndarray 또는 None: 합성된 이미지. 유효한 이미지가 없다면 None.
        """
        if not images:
            return None

        rows, cols = grid_shape
        max_cells = rows * cols
        width, height = tile_size

        valid_images = [img for img in images if img is not None]
        if not valid_images:
            return None

        tiles = []
        for idx in range(max_cells):
            if idx < len(images) and images[idx] is not None:
                tile = self._prepare_tile(images[idx], width, height)
            else:
                tile = np.full((height, width, 3), fill_color, dtype=np.uint8)
            tiles.append(tile)

        grid_rows = []
        for r in range(rows):
            row_tiles = tiles[r * cols:(r + 1) * cols]
            grid_rows.append(np.hstack(row_tiles))

        return np.vstack(grid_rows)

    @staticmethod
    def _prepare_tile(image, width, height):
        """
            개별 이미지를 컬러/크기 정규화
        """
        if len(image.shape) == 2:
            tile = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            tile = image.copy()
        return cv2.resize(tile, (width, height), interpolation=cv2.INTER_AREA)


if __name__ == "__main__":
    # AnalysisManager 인스턴스 생성 (비디오 레코딩, 이미지 변환, 그리드 합성 제공)
    manager = AnalysisManager()

    # 데모용 비디오 파일 경로 지정
    demo_video_path = os.path.join("recordings", "analysis_demo.avi")

    # 비디오 녹화 준비 (경로 설정 및 녹화 시작)
    manager.video_recorder.set_save_path(demo_video_path)
    manager.video_recorder.start_recording()

    for i in range(45):
        # 단일색 프레임 생성 (프레임별로 색 변화)
        frame = np.full((360, 640, 3), (i * 4) % 255, dtype=np.uint8)
        # 각 프레임마다 텍스트(프레임 번호) 표시
        cv2.putText(
            frame,
            f"Frame {i}",
            (50, 180),  # 텍스트 위치
            cv2.FONT_HERSHEY_SIMPLEX,
            1.5,       # 폰트 크기
            (255 - (i * 4) % 255, 255, 255),  # 폰트 색상(BGR)
            3,         # 두께
            cv2.LINE_AA,
        )
        # 비디오에 프레임 추가
        manager.video_recorder.add_frame(frame)

    # 비디오 녹화 종료(저장)
    manager.video_recorder.stop_recording()

    # ------------------------------------------------------------

    # 비디오 -> 이미지 변환: 프레임 하나씩 JPG로 저장 (analysis_frames 폴더)
    frames_dir = os.path.join("recordings", "analysis_frames")
    saved_images_path = manager.convert_video_to_images(
        demo_video_path,
        frames_dir,
        prefix="analysis",
        image_ext=".jpg",
    )

    # 변환된 이미지 중 앞에서 9장만 불러오기 (최대 9장 그리드용)
    sample_images = [cv2.imread(path) for path in saved_images_path[:9]]

    # 이미지를 3x3 그리드 형태로 합성 (타일 1개 크기 320x180)
    grid = manager.make_images_grid(sample_images, grid_shape=(3, 3), tile_size=(320, 180))

    # 그리드 이미지가 성공적으로 만들어졌으면 파일로 저장
    if grid is not None:
        grid_path = os.path.join("recordings", "analysis_grid.jpg")
        cv2.imwrite(grid_path, grid)