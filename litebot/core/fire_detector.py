#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
화재 감지 모듈
외부 시스템으로부터 불이 난 건물 번호를 텍스트 파일로 받아 상태를 관리합니다.
"""

import os
import time


class FireDetector(object):
    """
    파일 기반 화재 감지기
    fire_buildings.txt 파일을 감시하여 건물 번호를 읽고 상태를 저장합니다.
    형식: 띄어쓰기로 구분된 건물 번호 (예: "5" 또는 "2 7")
    """
    
    def __init__(self, file_path="fire_buildings.txt"):
        """
        FireDetector 초기화
        
        Args:
            file_path: 건물 번호가 담긴 텍스트 파일 경로
        """
        self.file_path = file_path
        
        # 9개 건물의 불 발생 상태 (1~9번 건물)
        # True: 불 발생, False: 정상
        self.buildings = {i: False for i in range(1, 10)}
    
    def set(self):
        """
        fire_buildings.txt 파일을 읽어서 건물 상태를 설정합니다.
        파일에 있는 건물 번호만 불이 난 것으로 설정하고, 나머지는 모두 False로 설정합니다.
        bot.step()에서 호출하면 됩니다.
        """
        # 먼저 모든 건물을 False로 초기화
        self.buildings = {i: False for i in range(1, 10)}
        
        if not os.path.exists(self.file_path):
            return
        
        try:
            with open(self.file_path, 'r') as f:
                line = f.readline().strip()
            
            if not line:
                # 빈 파일이면 모든 건물을 False로 유지
                return
            
            # 띄어쓰기로 구분된 건물 번호 파싱
            building_numbers = []
            for num_str in line.split():
                try:
                    building_number = int(num_str.strip())
                    if 1 <= building_number <= 9:
                        building_numbers.append(building_number)
                except ValueError:
                    pass
            
            if building_numbers:
                # 파일에 있는 건물 번호만 True로 설정
                for building_number in building_numbers:
                    self.buildings[building_number] = True
                print("[FireDetector] Set building status from file: {}".format(building_numbers))
            else:
                # 유효한 건물 번호가 없으면 모든 건물을 False로 유지
                print("[FireDetector] No valid building numbers found in file")
                
        except Exception as e:
            print("[FireDetector] Error reading file {}: {}".format(self.file_path, e))
    
    # ========== Getter 함수들 ==========
    
    def get_status(self, building_number=None):
        """
        건물 상태를 반환합니다.
        
        Args:
            building_number: 건물 번호 (1~9). None이면 전체 상태 반환
        
        Returns:
            bool 또는 dict: 
                - building_number가 지정되면 해당 건물의 불 발생 여부 (bool)
                - None이면 전체 건물 상태 딕셔너리 (dict)
        """
        if building_number is not None:
            if 1 <= building_number <= 9:
                return self.buildings.get(building_number, False)
            return False
        return self.buildings.copy()
    
    def get_fire_buildings(self):
        """
        불이 난 모든 건물 번호 리스트 반환
        
        Returns:
            list: 불이 난 건물 번호 리스트
        """
        return [num for num, status in self.buildings.items() if status]

