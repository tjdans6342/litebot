#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
파일 기반 객체 감지 워커

LiteBot이 저장한 이미지를 YYYYMMDD_HHMM_to_detect_images 폴더에서 읽고
YOLO 모델로 감지한 뒤 YYYYMMDD_HHMM_detected.log에 JSON 라인을 append 합니다.
또한 바운딩박스/라벨이 그려진 이미지를 YYYYMMDD_HHMM_detected 폴더에 저장합니다.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable

import cv2
from ultralytics import YOLO

try:
    import torch
except ImportError:  # pragma: no cover - optional dependency
    print("Warning: torch module not found. YOLO will use CPU.", file=sys.stderr)
    torch = None


IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png")
SESSION_INFO_FILENAME = "current_session.json"


def parse_args():
    parser = argparse.ArgumentParser(description="LiteBot 파일 기반 객체 감지기")
    parser.add_argument("--base-dir", default="detects", help="세션/로그를 저장할 기본 디렉터리")
    parser.add_argument("--model", default="config/best_rokaf.pt", help="YOLO 가중치 경로")
    parser.add_argument("--conf", type=float, default=0.3, help="confidence threshold")
    parser.add_argument("--sleep", type=float, default=1.0, help="폴링 주기 (초)")
    parser.add_argument(
        "--reuse-session",
        action="store_true",
        help="기존 current_session 정보를 재사용합니다. 없으면 새로 생성.",
    )
    return parser.parse_args()


def select_device():
    if torch and torch.cuda.is_available():
        device = 0
        print(f"[YOLO] Using GPU (device {device})")
    else:
        device = "cpu"
        print("[YOLO] Using CPU")
    return device


def create_or_load_session(base_dir: Path, reuse: bool) -> Dict[str, str]:
    base_dir.mkdir(parents=True, exist_ok=True)
    info_path = base_dir / SESSION_INFO_FILENAME

    if reuse and info_path.exists():
        return json.loads(info_path.read_text(encoding="utf-8"))

    session_ts = datetime.now().strftime("%Y%m%d_%H%M")
    # 세션 폴더 생성 (날짜_시간 형식)
    session_dir = _unique_path(base_dir, session_ts)
    session_dir.mkdir(parents=True, exist_ok=True)

    # 세션 폴더 안에 하위 폴더들 생성
    image_dir = session_dir / "to_detect_images"
    image_dir.mkdir(parents=True, exist_ok=True)

    log_path = session_dir / "detected.log"
    log_path.touch()

    output_dir = session_dir / "detected"
    output_dir.mkdir(parents=True, exist_ok=True)

    info = {
        "session_id": session_ts,
        "created_at": datetime.utcnow().isoformat() + "Z",
        "image_dir": str(image_dir.resolve()),
        "log_path": str(log_path.resolve()),
        "output_dir": str(output_dir.resolve()),
    }
    info_path.write_text(json.dumps(info, ensure_ascii=False, indent=2), encoding="utf-8")
    return info


def _unique_path(base_dir: Path, stem: str) -> Path:
    """
    동일 이름이 존재할 경우 _01, _02 ... 를 붙여 유일한 Path를 반환합니다.
    """
    candidate = base_dir / stem
    if not candidate.exists():
        return candidate

    counter = 1
    while True:
        candidate = base_dir / f"{stem}_{counter:02d}"
        if not candidate.exists():
            return candidate
        counter += 1


def iter_new_images(image_dir: Path, processed: set) -> Iterable[Path]:
    for path in sorted(image_dir.glob("*")):
        if path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue
        if path.name in processed:
            continue
        yield path


def run_loop(model: YOLO, device, image_dir: Path, log_path: Path, output_dir: Path, conf_threshold: float, sleep_sec: float):
    processed = set()
    class_counts = defaultdict(int)

    print(f"[YOLO] Watching directory: {image_dir}")
    print(f"[YOLO] Writing log: {log_path}")
    print(f"[YOLO] Writing annotated images to: {output_dir}")
    print("[YOLO] Press Ctrl+C to stop\n")

    while True:
        try:
            new_images = list(iter_new_images(image_dir, processed))
            if not new_images:
                time.sleep(sleep_sec)
                continue

            for img_path in new_images:
                img = cv2.imread(str(img_path))
                if img is None:
                    print(f"[WARN] Failed to read image: {img_path}")
                    processed.add(img_path.name)
                    continue

                results = model(img, verbose=False, device=device)
                per_image_counts = defaultdict(int)
                annotated = img.copy()

                for r in results:
                    boxes = getattr(r, "boxes", None)
                    if not boxes:
                        continue
                    for box in boxes:
                        conf = float(box.conf[0])
                        if conf < conf_threshold:
                            continue
                        class_id = int(box.cls[0])
                        label = model.names.get(class_id, str(class_id))
                        class_counts[label] += 1
                        per_image_counts[label] += 1
                        # draw bbox and label on annotated image
                        xyxy = box.xyxy[0].tolist()
                        x1, y1, x2, y2 = map(int, xyxy)
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        caption = f"{label} {conf:.2f}"
                        # background for text
                        (tw, th), baseline = cv2.getTextSize(caption, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                        ty1 = max(y1 - th - 4, 0)
                        cv2.rectangle(annotated, (x1, ty1), (x1 + tw + 2, ty1 + th + baseline + 2), (0, 255, 0), -1)
                        cv2.putText(annotated, caption, (x1 + 1, ty1 + th), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

                log_entry = {
                    "timestamp": datetime.utcnow().isoformat() + "Z",
                    "image_name": img_path.name,
                    "image_path": str(img_path.resolve()),
                    "counts": dict(per_image_counts),
                    "total_counts": dict(class_counts),
                }
                with log_path.open("a", encoding="utf-8") as fp:
                    fp.write(json.dumps(log_entry, ensure_ascii=False) + "\n")

                # save annotated image with the same filename
                out_path = output_dir / img_path.name
                try:
                    cv2.imwrite(str(out_path), annotated)
                except Exception as e:
                    print(f"[WARN] Failed to write annotated image: {out_path} ({e})")

                processed.add(img_path.name)
                _print_summary(img_path.name, per_image_counts, class_counts, conf_threshold)
        except KeyboardInterrupt:
            print("\n[YOLO] Stopped by user.")
            break
        except Exception as exc:  # pragma: no cover - defensive
            print(f"[ERROR] {exc}", file=sys.stderr)
            time.sleep(sleep_sec)


def _print_summary(image_name: str, per_image_counts: Dict[str, int], total_counts: Dict[str, int], conf: float):
    print(f"=== Image processed: {image_name} ===")
    if per_image_counts:
        print("  [Per-image counts]")
        for label, count in sorted(per_image_counts.items()):
            print(f"   - {label}: {count}")
    else:
        print(f"  No objects detected above conf {conf:.2f}")

    print("\n  [Cumulative counts so far]")
    for label, count in sorted(total_counts.items()):
        print(f"   - {label}: {count}")
    print("================================\n")


def main():
    args = parse_args()
    base_dir = Path(args.base_dir)
    session_info = create_or_load_session(base_dir, reuse=args.reuse_session)

    device = select_device()
    model = YOLO(args.model)

    image_dir = Path(session_info["image_dir"])
    log_path = Path(session_info["log_path"])
    output_dir = Path(session_info["output_dir"])
    run_loop(model, device, image_dir, log_path, output_dir, args.conf, args.sleep)


if __name__ == "__main__":
    main()

