#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Tiki 모드의 가장 단순한 LiteBot 실행 예제.
"""
import os
import sys
import time

# 프로젝트 루트를 Python path에 추가
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, PROJECT_ROOT)

from litebot.bot import LiteBot  # noqa: E402


def main():
    """
        Tiki 모드로 LiteBot 실행
    """
    print("[Tiki Example] Initializing LiteBot in Tiki mode...")
    bot = LiteBot(mode="tiki")
    
    print("[Tiki Example] Starting main loop...")
    print("[Tiki Example] Press Ctrl+C to stop")
    
    try:
        while True:
            resource_obs_pairs, actions, sources = bot.step()
            
            if resource_obs_pairs is None:
                time.sleep(0.05)
                continue
            
            # 모든 리소스별 액션 로그 출력
            for resource_type, action in actions.items():
                if action:
                    print("[LiteBot] resource={} action={} value={} source={}".format(
                        resource_type, action[0], action[1], sources.get(resource_type)))
            
            time.sleep(0.05)  # 20Hz
            
    except KeyboardInterrupt:
        print("\n[Tiki Example] Stopping...")
    finally:
        print("[Tiki Example] Done")


if __name__ == "__main__":
    main()

