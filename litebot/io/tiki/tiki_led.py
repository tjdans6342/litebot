#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    TikiLed 클래스
    Tiki 환경에서 LED를 제어하는 클래스
"""
try:
    from tiki.mini import TikiMini
    TIKI_AVAILABLE = True
except ImportError:
    TIKI_AVAILABLE = False
    print("[TikiLed] Warning: tiki.mini not available. Using stub implementation.")


class TikiLed:
    """
        Tiki 환경에서 LED를 제어하는 클래스
    """
    
    # LED 방향 상수
    DIRECTION_TOP = 0      # 상단 16개 LED
    DIRECTION_LEFT = 1     # 좌측 8개 LED
    DIRECTION_RIGHT = 2    # 우측 8개 LED
    
    def __init__(self):
        """
            TikiLed 초기화
        """
        if not TIKI_AVAILABLE:
            print("[TikiLed] Running in stub mode (tiki.mini not available)")
            self.tiki = None
        else:
            self.tiki = TikiMini()
    
    def set_led(self, direction, index, r, g, b):
        """
            LED 색상 설정
            
            Args:
                direction: LED 위치 (0=상단, 1=좌측, 2=우측)
                index: LED 번호 (상단: 0~15, 좌/우: 0~7)
                r: 빨간색 밝기 (0~255)
                g: 초록색 밝기 (0~255)
                b: 파란색 밝기 (0~255)
        """
        if self.tiki is None:
            print("[TikiLed] set_led({}, {}, {}, {}, {}) (stub)".format(
                direction, index, r, g, b))
            return
        
        self.tiki.set_led(direction, index, r, g, b)
    
    def clear_all(self):
        """
            모든 LED 끄기
        """
        # 상단 LED 끄기
        for i in range(16):
            self.set_led(self.DIRECTION_TOP, i, 0, 0, 0)
        
        # 좌측 LED 끄기
        for i in range(8):
            self.set_led(self.DIRECTION_LEFT, i, 0, 0, 0)
        
        # 우측 LED 끄기
        for i in range(8):
            self.set_led(self.DIRECTION_RIGHT, i, 0, 0, 0)
    
    def set_all_top(self, r, g, b):
        """
            상단 모든 LED 설정
            
            Args:
                r: 빨간색 밝기 (0~255)
                g: 초록색 밝기 (0~255)
                b: 파란색 밝기 (0~255)
        """
        for i in range(16):
            self.set_led(self.DIRECTION_TOP, i, r, g, b)
    
    def set_all_left(self, r, g, b):
        """
            좌측 모든 LED 설정
            
            Args:
                r: 빨간색 밝기 (0~255)
                g: 초록색 밝기 (0~255)
                b: 파란색 밝기 (0~255)
        """
        for i in range(8):
            self.set_led(self.DIRECTION_LEFT, i, r, g, b)
    
    def set_all_right(self, r, g, b):
        """
            우측 모든 LED 설정
            
            Args:
                r: 빨간색 밝기 (0~255)
                g: 초록색 밝기 (0~255)
                b: 파란색 밝기 (0~255)
        """
        for i in range(8):
            self.set_led(self.DIRECTION_RIGHT, i, r, g, b)
    
    def set_pattern(self, pattern, color):
        """
            LED 패턴 설정 (상단 LED만)
            
            Args:
                pattern: 패턴 이름 ('X', 'O', '#') 또는 LED 인덱스 리스트
                color: (r, g, b) 튜플
        """
        r, g, b = color
        
        if pattern == 'X':
            indices = [15, 0, 9, 6, 10, 5, 12, 3]
        elif pattern == 'O':
            indices = [15, 8, 7, 0, 12, 11, 4, 3, 14, 13, 1, 2]
        elif pattern == '#':
            indices = list(set([8, 9, 10, 11, 7, 6, 5, 4, 14, 9, 6, 1, 13, 10, 5, 2]))
        elif isinstance(pattern, list):
            indices = pattern
        else:
            print("[TikiLed] Unknown pattern: {}".format(pattern))
            return
        
        # 먼저 모두 끄기
        self.set_all_top(0, 0, 0)
        
        # 패턴 LED 켜기
        for i in indices:
            self.set_led(self.DIRECTION_TOP, i, r, g, b)

