#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
DetectionBridge 로그 읽기 테스트 (ultralytics 없이)
- 수동으로 로그 파일 생성 후 detection_bridge가 잘 읽는지 확인
"""
from __future__ import print_function

import json
import os
import sys
from datetime import datetime

# 프로젝트 루트를 경로에 추가
_script_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.abspath(os.path.join(_script_dir, '..'))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from ai.detection_bridge import DetectionBridge


def create_test_session(base_dir="detects"):
    """테스트용 세션 정보 파일 생성"""
    base_dir = os.path.abspath(base_dir)
    if not os.path.exists(base_dir):
        os.makedirs(base_dir)
    
    session_ts = datetime.now().strftime("%Y%m%d_%H%M")
    # 세션 폴더 생성 (날짜_시간 형식)
    session_dir = os.path.join(base_dir, session_ts)
    
    # 세션 폴더 안에 하위 폴더들 생성
    image_dir = os.path.join(session_dir, "to_detect_images")
    log_path = os.path.join(session_dir, "detected.log")
    output_dir = os.path.join(session_dir, "detected")
    
    # 디렉터리 생성
    os.makedirs(image_dir, exist_ok=True)
    os.makedirs(output_dir, exist_ok=True)
    
    # 로그 파일 생성 (비어있어도 됨)
    if not os.path.exists(log_path):
        open(log_path, 'w').close()
    
    # 세션 정보 파일 생성
    session_info = {
        "session_id": session_ts,
        "created_at": datetime.utcnow().isoformat() + "Z",
        "image_dir": os.path.abspath(image_dir),
        "log_path": os.path.abspath(log_path),
        "output_dir": os.path.abspath(output_dir),
    }
    
    session_info_path = os.path.join(base_dir, "current_session.json")
    with open(session_info_path, 'w', encoding='utf-8') as f:
        json.dump(session_info, f, ensure_ascii=False, indent=2)
    
    print("[Test] 세션 정보 파일 생성: {}".format(session_info_path))
    print("[Test] 세션 폴더: {}".format(session_dir))
    print("[Test] 이미지 디렉터리: {}".format(image_dir))
    print("[Test] 로그 파일: {}".format(log_path))
    
    return log_path, image_dir


def write_test_log_entry(log_path, image_name, counts, total_counts):
    """테스트용 로그 엔트리 작성"""
    log_entry = {
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "image_name": image_name,
        "image_path": os.path.abspath(os.path.join("detects", "test_images", image_name)),
        "counts": counts,
        "total_counts": total_counts,
    }
    
    with open(log_path, 'a', encoding='utf-8') as f:
        f.write(json.dumps(log_entry, ensure_ascii=False) + "\n")
    
    print("[Test] 로그 엔트리 추가: {}".format(image_name))


def test_read_logs():
    """DetectionBridge로 로그 읽기 테스트"""
    print("=" * 60)
    print("DetectionBridge 로그 읽기 테스트")
    print("=" * 60)
    
    # 1. 테스트 세션 생성
    print("\n[1] 테스트 세션 생성...")
    log_path, image_dir = create_test_session()
    
    # 2. 테스트 로그 엔트리 작성
    print("\n[2] 테스트 로그 엔트리 작성...")
    write_test_log_entry(log_path, "120000.jpg", {"pothole": 2, "person": 1}, {"pothole": 2, "person": 1})
    write_test_log_entry(log_path, "120001.jpg", {"pothole": 1}, {"pothole": 3, "person": 1})
    write_test_log_entry(log_path, "120002.jpg", {"aruco": 1}, {"pothole": 3, "person": 1, "aruco": 1})
    
    # 3. DetectionBridge로 읽기
    print("\n[3] DetectionBridge로 로그 읽기...")
    try:
        bridge = DetectionBridge()
        print("[OK] DetectionBridge 초기화 성공")
        print("    - 세션 ID: {}".format(bridge.session_id))
        print("    - 이미지 디렉터리: {}".format(bridge.image_dir))
        print("    - 로그 파일: {}".format(bridge.log_path))
        
        # 새 로그 읽기
        entries = bridge.read_new_logs()
        print("\n[OK] 로그 엔트리 {}개 읽기 성공".format(len(entries)))
        
        for i, entry in enumerate(entries, 1):
            print("\n  엔트리 {}:".format(i))
            print("    - 이미지: {}".format(entry.image_name))
            print("    - 타임스탬프: {}".format(entry.timestamp))
            print("    - 이번 이미지 counts: {}".format(entry.counts))
            print("    - 누적 counts: {}".format(entry.total_counts))
        
        # 최신 엔트리 확인
        latest = bridge.get_latest_counts()
        if latest:
            print("\n[OK] 최신 엔트리:")
            print("    - 이미지: {}".format(latest.image_name))
            print("    - 누적 counts: {}".format(latest.total_counts))
        
        # 4. 추가 로그 엔트리 작성 후 다시 읽기
        print("\n[4] 추가 로그 엔트리 작성 후 다시 읽기...")
        write_test_log_entry(log_path, "120003.jpg", {"pothole": 1, "aruco": 1}, {"pothole": 4, "person": 1, "aruco": 2})
        
        new_entries = bridge.read_new_logs()
        print("[OK] 새 로그 엔트리 {}개 읽기 성공".format(len(new_entries)))
        if new_entries:
            print("    - 새 이미지: {}".format(new_entries[-1].image_name))
            print("    - 새 누적 counts: {}".format(new_entries[-1].total_counts))
        
        print("\n" + "=" * 60)
        print("[SUCCESS] 모든 테스트 통과!")
        print("=" * 60)
        return True
        
    except Exception as e:
        print("\n[ERROR] 테스트 실패: {}".format(e))
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_read_logs()
    sys.exit(0 if success else 1)

