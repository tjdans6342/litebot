#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
get_largest_component 함수 테스트

HLS 필터링 → 이진화 → 연결된 컴포넌트 분석 → 가장 큰 컴포넌트만 남기기
"""

import sys
import os
import time
import cv2
import numpy as np

# 프로젝트 루트를 sys.path에 추가
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from litebot.utils import image_utils


def main():
    # 이미지 경로
    image_path = os.path.join(project_root, "recordings", "2024_maicon_photo.jpg")
    output_dir = os.path.join(project_root, "tests", "_out")
    
    # 출력 디렉터리 생성
    os.makedirs(output_dir, exist_ok=True)
    
    # 이미지 로드 (한글 경로 지원)
    print("[테스트] 이미지 로드 중: {}".format(image_path))
    if not os.path.exists(image_path):
        print("[오류] 이미지 파일을 찾을 수 없습니다: {}".format(image_path))
        return
    
    # Windows 한글 경로 문제 해결: cv2.imdecode 사용
    try:
        with open(image_path, 'rb') as f:
            image_data = np.frombuffer(f.read(), np.uint8)
        original = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
    except Exception as e:
        print("[오류] 이미지를 읽을 수 없습니다: {}".format(e))
        return
    
    if original is None:
        print("[오류] 이미지 디코딩 실패.")
        return
    
    print("[테스트] 이미지 크기: {}x{}".format(original.shape[1], original.shape[0]))
    
    # 1. HLS 필터링
    # HLS 값: H_min=50, H_max=180, L_min=70, L_max=100, S_min=0, S_max=25
    print("[테스트] HLS 필터링 적용 중...")
    hls_range = [[(50, 70, 0), (180, 100, 25)]]
    filtered_img = image_utils.color_filter(original, hls_range=hls_range, inverse=False)
    
    # 필터링 결과 저장 (한글 경로 지원)
    filtered_path = os.path.join(output_dir, "1_filtered_hls.jpg")
    success = cv2.imwrite(filtered_path, filtered_img)
    if not success:
        # 한글 경로 문제 해결: cv2.imencode 사용
        _, encoded = cv2.imencode('.jpg', filtered_img)
        with open(filtered_path, 'wb') as f:
            f.write(encoded.tobytes())
    print("[저장] HLS 필터링 결과: {}".format(filtered_path))
    
    # 2. 그레이스케일 변환
    print("[테스트] 그레이스케일 변환 중...")
    gray_img = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2GRAY)
    
    # 그레이스케일 결과 저장 (한글 경로 지원)
    gray_path = os.path.join(output_dir, "2_gray.jpg")
    success = cv2.imwrite(gray_path, gray_img)
    if not success:
        _, encoded = cv2.imencode('.jpg', gray_img)
        with open(gray_path, 'wb') as f:
            f.write(encoded.tobytes())
    print("[저장] 그레이스케일 결과: {}".format(gray_path))
    
    # 3. 이진화
    print("[테스트] 이진화 적용 중...")
    # 0이 아닌 픽셀을 255로 설정
    _, binary_img = cv2.threshold(gray_img, 1, 255, cv2.THRESH_BINARY)
    
    # 이진화 결과 저장 (한글 경로 지원)
    binary_path = os.path.join(output_dir, "3_binary.jpg")
    success = cv2.imwrite(binary_path, binary_img)
    if not success:
        _, encoded = cv2.imencode('.jpg', binary_img)
        with open(binary_path, 'wb') as f:
            f.write(encoded.tobytes())
    print("[저장] 이진화 결과: {}".format(binary_path))
    
    # 이진화 전 픽셀 개수 확인
    white_pixels_before = np.sum(binary_img == 255)
    print("[정보] 이진화 후 흰색 픽셀 개수: {}".format(white_pixels_before))
    
    # 4. 연결된 컴포넌트 분석 (가장 큰 컴포넌트만 남기기)
    print("[테스트] 연결된 컴포넌트 분석 중 (8방향 연결성)...")
    start_time = time.time()
    result_img, component_count, largest_size = image_utils.get_largest_component(
        binary_img, 
        connectivity=8
    )
    elapsed_time = time.time() - start_time
    print("[정보] 분석 완료 (소요 시간: {:.2f}초)".format(elapsed_time))
    print("[정보] 발견된 컴포넌트 개수: {}".format(component_count))
    print("[정보] 가장 큰 컴포넌트 크기: {} 픽셀".format(largest_size))
    print("[정보] 가장 큰 컴포넌트 비율: {:.2f}%".format(
        (largest_size / white_pixels_before * 100) if white_pixels_before > 0 else 0
    ))
    
    # 최종 결과 저장 (한글 경로 지원)
    result_path = os.path.join(output_dir, "4_largest_component.jpg")
    success = cv2.imwrite(result_path, result_img)
    if not success:
        _, encoded = cv2.imencode('.jpg', result_img)
        with open(result_path, 'wb') as f:
            f.write(encoded.tobytes())
    print("[저장] 가장 큰 컴포넌트 결과: {}".format(result_path))
    
    # 5. 비교를 위한 시각화 (모든 이미지를 원본 크기로 통일)
    print("[테스트] 비교 이미지 생성 중...")
    
    # 원본 이미지 크기
    h, w = original.shape[:2]
    
    # 모든 이미지를 원본 크기로 리사이즈
    filtered_resized = cv2.resize(filtered_img, (w, h))
    gray_bgr = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)
    gray_resized = cv2.resize(gray_bgr, (w, h))
    binary_bgr = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
    binary_resized = cv2.resize(binary_bgr, (w, h))
    result_bgr = cv2.cvtColor(result_img, cv2.COLOR_GRAY2BGR)
    result_resized = cv2.resize(result_bgr, (w, h))
    
    # 상하로 합치기
    top_row = np.hstack([original, filtered_resized])
    bottom_row = np.hstack([gray_resized, binary_resized])
    comparison = np.vstack([top_row, bottom_row])
    
    # comparison의 높이에 맞춰 result_resized 리사이즈
    comp_h, comp_w = comparison.shape[:2]
    result_resized_final = cv2.resize(result_resized, (comp_w, comp_h))
    
    # 최종 결과를 오른쪽에 추가
    comparison_final = np.hstack([comparison, result_resized_final])
    
    comparison_path = os.path.join(output_dir, "5_comparison.jpg")
    success = cv2.imwrite(comparison_path, comparison_final)
    if not success:
        # 한글 경로 문제 해결: cv2.imencode 사용
        _, encoded = cv2.imencode('.jpg', comparison_final)
        with open(comparison_path, 'wb') as f:
            f.write(encoded.tobytes())
    print("[저장] 비교 이미지: {}".format(comparison_path))
    
    print("\n[완료] 모든 결과가 {} 폴더에 저장되었습니다.".format(output_dir))
    print("  - 1_filtered_hls.jpg: HLS 필터링 결과")
    print("  - 2_gray.jpg: 그레이스케일 변환 결과")
    print("  - 3_binary.jpg: 이진화 결과")
    print("  - 4_largest_component.jpg: 가장 큰 컴포넌트만 남긴 결과")
    print("  - 5_comparison.jpg: 전체 과정 비교 이미지")


if __name__ == "__main__":
    main()

