#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
FireDetector 테스트
1초마다 fire_buildings.txt 파일을 읽어서 현재 불이 난 건물 상태를 출력합니다.
fire_buildings.txt 파일을 수정하면 실시간으로 반영되는지 확인할 수 있습니다.
"""
from __future__ import print_function

import os
import sys
import time

# 프로젝트 루트를 경로에 추가
_script_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.abspath(os.path.join(_script_dir, '..'))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from litebot.core.fire_detector import FireDetector


def main():
    """FireDetector 간단 테스트"""
    test_file = "fire_buildings.txt"
    
    print("=" * 60)
    print("FireDetector 테스트")
    print("=" * 60)
    print("1초마다 fire_buildings.txt 파일을 읽어서 불이 난 건물 상태를 출력합니다.")
    print("fire_buildings.txt 파일을 수정하면 실시간으로 반영됩니다.")
    print("종료하려면 Ctrl+C를 누르세요.")
    print("=" * 60)
    print()
    
    # FireDetector 초기화
    detector = FireDetector(file_path=test_file)
    
    try:
        while True:
            # 파일 읽어서 상태 설정
            detector.set()
            
            # 현재 불이 난 건물 상태 출력
            fire_buildings = detector.get_fire_buildings()
            status = detector.get_status()
            
            print("[{}] 불이 난 건물: {}".format(
                time.strftime("%H:%M:%S"),
                fire_buildings if fire_buildings else "없음"
            ))
            print("    전체 상태: {}".format(status))
            
            # 파일 존재 여부 확인
            if os.path.exists(test_file):
                with open(test_file, 'r', encoding='utf-8') as f:
                    file_content = f.read().strip()
                print("    파일 내용: '{}'".format(file_content if file_content else "(빈 파일)"))
            else:
                print("    파일 상태: 존재하지 않음")
            
            print()
            
            # 2초 대기
            time.sleep(2.0)
            
    except KeyboardInterrupt:
        print("\n[종료] 테스트를 종료합니다.")


if __name__ == "__main__":
    main()
