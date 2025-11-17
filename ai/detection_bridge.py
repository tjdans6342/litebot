#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
LiteBot <-> 외부 객체 감지 파이프라인 브리지 (Python 2 호환)
"""

import io
import json
import os
from datetime import datetime

import cv2


class DetectionLogEntry(object):
    def __init__(self, timestamp, image_name, image_path, counts, total_counts):
        self.timestamp = timestamp or ""
        self.image_name = image_name or ""
        self.image_path = image_path or ""
        self.counts = counts or {}
        self.total_counts = total_counts or {}


class DetectionBridge(object):
    IMAGE_EXT = ".jpg"

    def __init__(self, base_dir="detects"):
        self.base_dir = os.path.abspath(base_dir)
        self.session_info_path = os.path.join(self.base_dir, "current_session.json")
        self.session_id = None
        self.image_dir = None
        self.log_path = None
        self._log_position = 0
        self.refresh_session()

    # ------------------------------------------------------------------ #
    def refresh_session(self):
        if not os.path.exists(self.session_info_path):
            raise IOError(
                "세션 정보 파일({})을 찾을 수 없습니다. object_detector.py가 실행 중인지 확인하세요.".format(
                    self.session_info_path
                )
            )

        with io.open(self.session_info_path, "r", encoding="utf-8") as fp:
            data = json.load(fp)

        self.session_id = data.get("session_id")
        if not self.session_id:
            raise ValueError("session_id가 세션 정보에 없습니다.")

        img_dir = data.get("image_dir")
        log_path = data.get("log_path")
        if img_dir is None or log_path is None:
            raise ValueError("session 정보에 image_dir/log_path가 없습니다.")

        self.image_dir = self._resolve_path(img_dir)
        self.log_path = self._resolve_path(log_path)
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)

        self._log_position = 0

    def _resolve_path(self, path_str):
        if os.path.isabs(path_str):
            return path_str
        return os.path.abspath(os.path.join(self.base_dir, path_str))

    # ------------------------------------------------------------------ #
    def save_frame(self, frame, suffix=None):
        if self.image_dir is None:
            raise RuntimeError("세션이 초기화되지 않았습니다. refresh_session()을 호출하세요.")

        timestamp = datetime.now().strftime("%H%M%S")
        if suffix:
            safe_suffix = str(suffix).replace(os.sep, "_")
            filename = "{}_{}{}".format(timestamp, safe_suffix, self.IMAGE_EXT)
        else:
            filename = "{}{}".format(timestamp, self.IMAGE_EXT)

        save_path = os.path.join(self.image_dir, filename)
        counter = 1
        while os.path.exists(save_path):
            root, ext = os.path.splitext(filename)
            save_path = os.path.join(self.image_dir, "{}_{:02d}{}".format(root, counter, ext))
            counter += 1

        success = cv2.imwrite(save_path, frame)
        if not success:
            raise IOError("프레임을 저장하지 못했습니다: {}".format(save_path))
        return save_path

    # ------------------------------------------------------------------ #
    def read_new_logs(self):
        if self.log_path is None or not os.path.exists(self.log_path):
            return []

        entries = []
        with io.open(self.log_path, "r", encoding="utf-8") as fp:
            fp.seek(self._log_position)
            for raw in fp:
                line = raw.strip()
                if not line:
                    continue
                try:
                    payload = json.loads(line)
                    entry = DetectionLogEntry(
                        payload.get("timestamp"),
                        payload.get("image_name"),
                        payload.get("image_path"),
                        payload.get("counts"),
                        payload.get("total_counts"),
                    )
                    entries.append(entry)
                except ValueError:
                    continue
            self._log_position = fp.tell()
        return entries

    def get_latest_counts(self):
        entries = self.read_new_logs()
        return entries[-1] if entries else None

