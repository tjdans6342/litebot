#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    OpenCV VideoWriter 기반 비디오 레코더
"""
import os
from datetime import datetime

import cv2
import numpy as np


class VideoRecorder:
    """
        프레임을 받아 비디오 파일로 저장하는 헬퍼 클래스
    """

    def __init__(self, save_path=None, fps=20.0, codec="XVID"):
        """
            VideoRecorder 초기화

            Args:
                save_path (str, optional):
                    출력 비디오 경로. None이면 start_recording 호출 시 timestamp 기반으로 생성.
                fps (float):
                    출력 영상의 초당 프레임 수.
                codec (str):
                    OpenCV VideoWriter_fourcc 코드. 기본 "XVID".
        """
        self.save_path = save_path
        self.fps = fps
        self.codec = codec
        self._writer = None
        self._frame_size = None
        self._recording = False

    def set_save_path(self, save_path):
        """
            비디오 저장 경로를 설정합니다.
        """
        self.save_path = save_path

    def start_recording(self):
        """
            녹화를 시작합니다. 실제 VideoWriter는 첫 프레임에서 초기화됩니다.
        """
        if self._recording:
            return
        if self.save_path is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.save_path = os.path.join("recordings", "litebot_{}.avi".format(timestamp))
        save_dir = os.path.dirname(self.save_path)
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        self._recording = True

    def add_frame(self, frame):
        """
            프레임을 VideoWriter에 추가합니다.

            Args:
                frame (numpy.ndarray): BGR 또는 Grayscale 프레임
        """
        if not self._recording:
            raise RuntimeError("Recording has not started. Call start_recording() first.")
        if frame is None:
            return
        frame = self._ensure_color_frame(frame)
        if self._writer is None:
            self._init_writer(frame.shape[1], frame.shape[0])
        self._writer.write(frame)

    def stop_recording(self):
        """
            녹화를 종료하고 리소스를 해제합니다.
        """
        self._release_writer()
        self._frame_size = None
        self._recording = False

    def is_recording(self):
        """
            현재 녹화 중인지 여부를 반환합니다.
        """
        return self._recording

    def _init_writer(self, width, height):
        """
            VideoWriter를 초기화합니다.
        """
        fourcc = cv2.VideoWriter_fourcc(*self.codec)
        self._frame_size = (width, height)
        self._writer = cv2.VideoWriter(self.save_path, fourcc, self.fps, self._frame_size)
        if not self._writer.isOpened():
            self._writer = None
            raise IOError("Failed to open VideoWriter at {}".format(self.save_path))

    # @staticmethod는 인스턴스(self)나 클래스(cls)를 인자로 받지 않는 독립적인 유틸리티 함수를 클래스 안에 정의할 때 씁니다.
    # 그래서 VideoRecorder._ensure_color_frame(frame)처럼 클래스 이름으로 부를 수 있습니다.
    # 접두사로 _를 붙이는 건 "외부에서 직접 쓰지 마라, 내부(프라이빗)용이다"라는 관례일 뿐 막는 기능은 없습니다.
    # 즉, 내부에서만 쓴다는 의도와, 인스턴스 없이 클래스.함수로 호출 가능하다는 점이 모두 적용됩니다. 의미 충돌은 없습니다.
    # "_붙음+staticmethod" 조합은 "이 클래스의 헬퍼로나 (외부에서 직접 말고) 내부 코드에서, 인스턴스 없이도 자유롭게 쓸 수 있는 함수"라는 뜻입니다.
    @staticmethod
    def _ensure_color_frame(frame):
        """
            그레이스케일 프레임을 BGR로 변환합니다.
        """
        if not isinstance(frame, np.ndarray):
            raise TypeError("Frame must be a numpy.ndarray")
        if len(frame.shape) == 2:
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        return frame

    def _release_writer(self):
        """
            내부 VideoWriter 리소스를 해제합니다.
        """
        if self._writer is not None:
            self._writer.release()
            self._writer = None

    def __del__(self):
        self._release_writer()


if __name__ == "__main__":
    recorder = VideoRecorder()
    recorder.set_save_path("./recordings/demo.avi")
    recorder.start_recording()

    dummy_frames = [
        np.full((480, 640, 3), fill_value=i * 8, dtype=np.uint8) for i in range(60)
    ]

    for frame in dummy_frames:
        recorder.add_frame(frame)

    recorder.stop_recording()

