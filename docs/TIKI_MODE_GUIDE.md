# Tiki 모드 사용 가이드

## 개요

LiteBot은 `mode="tiki"`로 설정하면 Tiki 하드웨어를 사용할 수 있습니다. 이 문서는 Tiki 모드를 사용하기 전에 확인하고 설정해야 할 사항들을 정리합니다.

## 빠른 시작

```python
from litebot.bot import LiteBot

# Tiki 모드로 초기화
bot = LiteBot(mode="tiki")

# 나머지는 ROS 모드와 동일하게 사용
resource_obs_pairs, actions, sources = bot.step()
```

## 필수 사항

### 1. Tiki 라이브러리 설치

Tiki 라이브러리가 설치되어 있어야 실제 하드웨어 제어가 가능합니다.

#### 설치 확인
```python
try:
    from tiki.mini import TikiMini
    print("Tiki 라이브러리 사용 가능")
except ImportError:
    print("Tiki 라이브러리 없음 - stub 모드로 동작")
```

#### 설치 경로
일반적으로 Jetson Nano에 다음 경로에 설치됩니다:
- `/home/jetson/Setup/venv/lib/python3.8/site-packages/tiki/mini/__init__.py`

#### Stub 모드
Tiki 라이브러리가 없으면 자동으로 stub 모드로 동작합니다:
- 실제 하드웨어 제어는 안 됨
- 에러 없이 동작 (테스트/개발용)
- 콘솔에 메시지만 출력

### 2. 하드웨어 환경

#### 필수 하드웨어
- **Tiki 로봇**: 실제 Tiki 하드웨어 연결 필요
- **Jetson Nano**: 카메라 사용 시 필요
- **CSI 카메라**: Jetson Nano에 연결된 카메라

#### 카메라 설정
- 기본값: 640x480 @ 30fps
- GStreamer 파이프라인 사용
- Jetson Nano 환경에서만 동작

## 캘리브레이션 (실제 하드웨어 사용 시 필수)

실제 로봇에서 정확한 제어를 위해 다음 값들을 조정해야 합니다.

### 1. 인코더 캘리브레이션

**파일**: `litebot/io/tiki/tiki_controller.py`

```python
# 현재 기본값 (수정 필요)
self.encoder_ticks_per_meter = 1000  # 1미터당 인코더 틱 수
```

#### 측정 방법
1. 로봇을 정확히 1미터 전진시킴
2. 인코더 값 읽기
3. 계산: `encoder_ticks_per_meter = 인코더_값 / 1.0`

#### 예시
```python
from litebot.io.tiki import TikiSensor

sensor = TikiSensor()
initial = sensor.get_encoder('LEFT')
# 로봇을 1미터 전진
final = sensor.get_encoder('LEFT')
ticks_per_meter = final - initial
print(f"1미터당 인코더 틱: {ticks_per_meter}")
```

### 2. 바퀴 간 거리 (Wheelbase)

**파일**: `litebot/io/tiki/tiki_controller.py` → `_convert_angular_to_differential()`

```python
# 현재 기본값 (수정 필요)
wheelbase = 0.2  # 바퀴 간 거리 (m)
```

#### 측정 방법
- 실제 로봇의 좌우 바퀴 중심 간 거리를 미터 단위로 측정

### 3. 최대 속도

**파일**: `litebot/io/tiki/tiki_controller.py` → `_convert_speed_to_rpm()`

```python
# 현재 기본값 (수정 필요)
max_speed_mps = 1.0  # 최대 속도 (m/s)
```

#### 측정 방법
1. 로봇을 최대 속도로 전진
2. 일정 시간 동안 이동 거리 측정
3. 계산: `max_speed_mps = 이동거리 / 시간`

#### 예시
```python
from litebot.io.tiki import TikiController
import time

ctrl = TikiController()
# 최대 속도로 전진
ctrl.update_speed_angular(1.0, 0.0)  # 1.0 m/s 가정
time.sleep(2.0)  # 2초
# 실제 이동 거리 측정 후 계산
```

### 4. 모터 모드 설정

**기본값**: PID 모드

```python
# PID 모드 (정밀 제어) - 기본값
ctrl = TikiController(motor_mode='PID')

# PWM 모드 (단순 제어)
ctrl = TikiController(motor_mode='PWM')
```

#### 모드 선택 가이드
- **PID 모드**: 정밀한 속도/위치 제어 필요 시
- **PWM 모드**: 간단한 제어만 필요 시

## 설정 파일 수정 가이드

### 방법 1: 코드 직접 수정

`litebot/io/tiki/tiki_controller.py` 파일을 직접 수정:

```python
# __init__ 메서드에서
self.encoder_ticks_per_meter = 1500  # 실제 측정값

# _convert_angular_to_differential 메서드에서
wheelbase = 0.25  # 실제 측정값

# _convert_speed_to_rpm 메서드에서
max_speed_mps = 0.8  # 실제 측정값
```

### 방법 2: 설정 파일 생성 (권장)

새로운 설정 파일을 만들어서 사용:

```python
# litebot/configs/tiki_config.py
TIKI_CONFIG = {
    'encoder_ticks_per_meter': 1500,
    'wheelbase': 0.25,  # m
    'max_speed_mps': 0.8,  # m/s
    'motor_mode': 'PID',
}
```

그리고 `TikiController`에서 이 설정을 읽도록 수정.

## 테스트 및 검증

### 1. 기본 동작 테스트

```bash
# 기본 실행 테스트
python examples/tiki/1_run_simple.py

# 센서 테스트
python examples/tiki/2_test_sensors.py

# LED 테스트
python examples/tiki/3_test_led.py
```

### 2. 거리 제어 정확도 테스트

```python
from litebot.io.tiki import TikiController, TikiSensor
import time

ctrl = TikiController()
sensor = TikiSensor()

# 1미터 전진 테스트
initial = sensor.get_encoder('LEFT')
ctrl.drive_forward_distance(1.0, 0.3)  # 1m, 0.3 m/s
final = sensor.get_encoder('LEFT')

actual_distance = (final - initial) / ctrl.encoder_ticks_per_meter
print(f"목표: 1.0m, 실제: {actual_distance:.3f}m")
print(f"오차: {abs(1.0 - actual_distance) * 100:.1f}%")
```

### 3. 각속도 제어 테스트

```python
from litebot.io.tiki import TikiController
import time

ctrl = TikiController()

# 90도 회전 테스트
ctrl.rotate_in_place(90.0, ang_speed=1.0)
# 실제 회전 각도 확인 (IMU 또는 시각적 확인)
```

## 문제 해결

### 문제 1: Tiki 라이브러리를 찾을 수 없음

**증상**:
```
[TikiController] Warning: tiki.mini not available. Using stub implementation.
```

**해결**:
1. Tiki 라이브러리 설치 경로 확인
2. Python 경로에 추가:
   ```python
   import sys
   sys.path.append('/home/jetson/Setup/venv/lib/python3.8/site-packages')
   ```

### 문제 2: 거리 제어가 부정확함

**증상**: 목표 거리와 실제 이동 거리가 다름

**해결**:
1. `encoder_ticks_per_meter` 값 재측정
2. 인코더 값이 음수일 수 있으므로 절댓값 사용 고려
3. 바퀴 미끄러짐 고려 (실제 환경에서 재측정)

### 문제 3: 회전이 부정확함

**증상**: 목표 각도와 실제 회전 각도가 다름

**해결**:
1. `wheelbase` 값 재측정
2. 회전 속도(`ang_speed`) 조정
3. IMU 센서로 실제 각도 확인

### 문제 4: 카메라가 동작하지 않음

**증상**: `get_frame()`이 None 반환

**해결**:
1. Jetson Nano 환경인지 확인
2. CSI 카메라 연결 확인
3. GStreamer 설치 확인:
   ```bash
   gst-launch-1.0 --version
   ```
4. 카메라 권한 확인

### 문제 5: 속도가 너무 느리거나 빠름

**증상**: 명령한 속도와 실제 속도가 다름

**해결**:
1. `max_speed_mps` 값 재측정
2. `_convert_speed_to_rpm()` 함수의 변환 공식 확인
3. 모터 모드 확인 (PWM vs PID)

## 성능 최적화

### 1. 인코더 폴링 주기

거리 제어 시 인코더를 읽는 주기를 조정:

```python
# tiki_controller.py의 drive_forward_distance 등에서
time.sleep(0.01)  # 10ms 간격 (기본값)
# 더 빠른 응답이 필요하면 0.005 (5ms)로 줄임
# CPU 부하가 높으면 0.02 (20ms)로 늘림
```

### 2. 모터 모드 선택

- **PID 모드**: 정밀 제어 필요 시 (기본값)
- **PWM 모드**: 빠른 응답 필요 시

### 3. 카메라 해상도

```python
# 낮은 해상도로 변경하여 성능 향상
cam = TikiCamera(width=320, height=240, framerate=30)
```

## ROS 모드와의 차이점

| 항목 | ROS 모드 | Tiki 모드 |
|------|----------|-----------|
| **초기화** | `rospy.init_node()` 필요 | 불필요 |
| **시간 함수** | `rospy.sleep()`, `rospy.Rate()` | `time.sleep()` |
| **통신** | ROS 토픽 (메시지 기반) | 직접 함수 호출 |
| **설정** | ROS 파라미터 | 코드 내 상수 |
| **환경** | ROS 환경 필요 | Tiki 라이브러리만 필요 |

## 체크리스트

실제 하드웨어에서 Tiki 모드를 사용하기 전에:

- [ ] Tiki 라이브러리 설치 확인
- [ ] 하드웨어 연결 확인
- [ ] `encoder_ticks_per_meter` 측정 및 설정
- [ ] `wheelbase` 측정 및 설정
- [ ] `max_speed_mps` 측정 및 설정
- [ ] 모터 모드 선택 (PWM/PID)
- [ ] 기본 동작 테스트 완료
- [ ] 거리 제어 정확도 검증
- [ ] 각속도 제어 정확도 검증
- [ ] 카메라 동작 확인

## 추가 리소스

- **예제 코드**: `examples/tiki/`
- **테스트 코드**: `tests/hardware/tiki_*.py`
- **API 문서**: `litebot/io/tiki/` 디렉토리의 각 모듈
- **Tiki 라이브러리 문서**: Tiki 공식 문서 참조

## 문의 및 지원

문제가 발생하면:
1. 이 가이드의 문제 해결 섹션 확인
2. 테스트 코드로 개별 컴포넌트 확인
3. 로그 메시지 확인 (stub 모드인지 실제 모드인지)

---

**마지막 업데이트**: 2024년
**버전**: 1.0

