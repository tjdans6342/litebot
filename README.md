```
litebot/
├── litebot/                       
│   ├── core/                    
│   │   ├── observer.py          
│   │   ├── trigger_manager.py   
│   │   ├── trigger_lane.py
│   │   ├── trigger_aruco.py
│   │   ├── trigger_pothole.py
│   │   ├── trigger_qrcode.py
│   │   └── fire_detector.py    # 화재 건물 상태 감지 (파일 기반)
│   │
│   ├── action/                  
│   │   └── action_executor.py
│   │
│   ├── io/                      
│   │   ├── camera_interface.py
│   │   ├── controller_interface.py
│   │   ├── ros/                 
│   │   │   ├── ros_camera.py
│   │   │   └── ros_controller.py
│   │   └── tiki/                
│   │       ├── tiki_camera.py
│   │       ├── tiki_controller.py
│   │       ├── tiki_sensor.py
│   │       ├── tiki_led.py
│   │       └── tiki_oled.py
│   │
│   ├── processing/              
│   │   ├── image_processor.py
│   │   └── image_yolo.py
│   │
│   ├── analysis/                # 주행 데이터, 분석 기록용
│   │   ├── analysis_manager.py
│   │   ├── video_recorder.py
│   │   ├── hls_viewer.py        # HLS/RGB 범위 분석 도구
│   │   └── manual_drive.py     # 수동 주행 테스트 도구
│   │
│   ├── configs/                 # 감지, 추론, 주행 설정 등
│   │   ├── lane_config.py
│   │   ├── aruco_rules.py
│   │   └── video_config.py
│   │
│   └── utils/                   
│       ├── aruco_utils.py
│       ├── check_utils.py
│       └── image_utils.py       # BEV, color_filter, get_largest_component 등
│
├── examples/
│   ├── ros/
│   │   ├── run_ros.py                  # 기본 실행 (튜토리얼 1단계와 동일)
│   │   ├── 1_run_simple.py             # 튜토리얼 1: 최소 구성
│   │   ├── 2_run_with_video.py         # 튜토리얼 2: 주행 + 비디오 녹화
│   │   ├── 3_run_parallel_obs.py       # 튜토리얼 3: 병렬 관측 처리
│   │   └── 4_run_with_detection.py     # 튜토리얼 4: DetectionBridge 연계
│   └── tiki/
│       ├── 1_run_simple.py      # 기본 실행
│       ├── 2_test_sensors.py     # 센서 테스트
│       └── 3_test_led.py          # LED 테스트
│
├── ai/
│   ├── detection_bridge.py
│   └── object_detector.py
│
├── models/                    # YOLO 등 추론 가중치(.pt) 보관
│   └── (예: best_rokaf.pt)
│
├── tests/                       
│   ├── software/                # 소프트웨어 로직 테스트
│   │   ├── manual_check.py
│   │   ├── test_resource_separation.py
│   │   └── test_detection_bridge.py
│   ├── hardware/                # 하드웨어 연동 테스트
│   │   ├── ros_controller_run.py
│   │   ├── ros_executor_run.py
│   │   ├── ros_resource_separation.py
│   │   ├── ros_camera_run.py
│   │   └── tiki_*.py (동일 구조)
│   ├── integration/             # 통합 테스트
│   │   ├── capture_pothole.py
│   │   └── capture_aruco.py
│   ├── test_largest_component.py
│   └── test_fire_detector.py
│
├── README.md
├── setup.py
└── requirements.txt
```

## 주요 기능

### 1. 리소스 기반 비동기 액션 실행 시스템

- **리소스 타입별 액션 분류**: motor, led, oled 등으로 분류하여 독립적인 실행 제어
- **비동기 실행**: 장시간 액션(drive_forward, rotate 등)은 스레드에서 실행하여 실시간 프레임 처리 보장
- **리소스별 상호 배제**: 같은 리소스 타입의 액션이 실행 중이면 새 액션 무시 (큐 방식)
- **동시 실행 지원**: 서로 다른 리소스 타입의 액션은 동시 실행 가능

### 2. 이미지 처리 파이프라인

- **BEV 변환**: 원근 보정을 통한 차선 감지 정확도 향상
- **HLS/RGB 이중 필터링**: 조명 변화에 강건한 색상 필터링
- **연결된 컴포넌트 분석**: `get_largest_component()` 함수로 도로 영역만 추출하고 노이즈 자동 제거
- **파이프라인**: `BEV → HLS/RGB 필터링 → 그레이스케일 → 블러 → 임계값 → 연결된 컴포넌트 분석 → Canny → Hough Line`

### 3. 객체 감지 파이프라인

- `ai/object_detector.py`  
  - 실행 시 `detects/YYYYMMDD_HHMM` 세션 폴더를 생성하고, 그 안에 `to_detect_images/`, `detected/`, `detected.log`를 자동 생성합니다.  
  - LiteBot이 `to_detect_images/` 폴더에 저장한 이미지를 주기적으로 스캔하고 YOLO 추론 결과를 `detected.log`(JSON Lines)로 남깁니다.  
  - 현재 세션 정보는 `detects/current_session.json`에 기록되며, 다른 프로세스가 참조할 수 있습니다.
  - 감지된 바운딩박스/라벨이 그려진 이미지는 `detected/` 폴더에 원본 파일명 그대로 저장됩니다.

- `ai/detection_bridge.py`  
  - LiteBot 측에서 `current_session.json`을 읽어 최신 세션을 추적하고, 프레임을 HHMMSS.jpg 형태로 저장하거나 새로 추가된 감지 로그를 스트리밍할 수 있는 헬퍼 클래스입니다.

### 4. 분석 및 디버깅 도구

- **HLS/RGB Viewer** (`litebot/analysis/hls_viewer.py`)
  - 비디오/이미지 파일에서 HLS 및 RGB 범위를 실시간으로 조정하며 분석
  - 트랙바로 H/L/S, R/G/B 최소·최대값 조절
  - HLS와 RGB 마스크를 AND 연산으로 결합하여 더 정확한 필터링
  - 검은색 도로 감지를 위한 이중 필터링 지원

- **Manual Drive** (`litebot/analysis/manual_drive.py`)
  - 키보드로 로봇을 수동 제어하며 테스트
  - 실시간 카메라 프레임 표시
  - 이미지 캡처 기능

### 5. 테스트 구조

- **Software 테스트** (`tests/software/`): 로직 검증 (DummyController 사용)
- **Hardware 테스트** (`tests/hardware/`): 실제 하드웨어 연동 검증
- **Integration 테스트** (`tests/integration/`): 전체 시스템 통합 테스트

## 실행 예시

### 객체 감지 워커

```bash
# 객체 감지 워커 실행 (가중치는 models/ 아래에 위치)
# --conf  : YOLO confidence threshold (예: 0.35)
# --sleep : 폴더 폴링 주기 (초 단위)
python ai/object_detector.py --model models/best_rokaf.pt --conf 0.35 --sleep 0.5
```

### LiteBot에서 감지 세션 사용

```python
from ai.detection_bridge import DetectionBridge

bridge = DetectionBridge()
bridge.save_frame(frame)
logs = bridge.read_new_logs()
```

### HLS/RGB 범위 분석

```bash
# 비디오 파일 분석
python -m litebot.analysis.hls_viewer --video recordings/example_video.mp4

# 이미지 파일 분석
python -m litebot.analysis.hls_viewer --video recordings/2024_maicon_photo.jpg

# 스케일 조정
python -m litebot.analysis.hls_viewer --video recordings/example_video.mp4 --scale 0.5
```

### 수동 주행 테스트

```bash
python -m litebot.analysis.manual_drive --linear 0.2 --angular 0.5
```

## Tiki 모드 사용법

### Tiki 라이브러리 설치

Tiki 라이브러리는 Jetson Nano에 이미 설치되어 있어야 합니다. 
만약 설치되지 않았다면, 다음 경로에 설치되어 있는지 확인하세요:
- `/home/jetson/Setup/venv/lib/python3.8/site-packages/tiki/mini/__init__.py`

### 기본 실행

```bash
# Tiki 모드로 LiteBot 실행
python examples/tiki/1_run_simple.py
```

### 센서 테스트

```bash
# 배터리, IMU, 인코더 값 읽기
python examples/tiki/2_test_sensors.py
```

### LED 테스트

```bash
# LED 패턴 표시
python examples/tiki/3_test_led.py
```

### Python 코드에서 사용

```python
from litebot.bot import LiteBot

# Tiki 모드로 초기화
bot = LiteBot(mode="tiki")

# 센서 직접 사용
from litebot.io.tiki import TikiSensor, TikiLed, TikiOled

sensor = TikiSensor()
voltage = sensor.get_battery_voltage()
ax, ay, az = sensor.get_imu()

led = TikiLed()
led.set_pattern('X', (50, 0, 0))  # 빨간색 X 표시

oled = TikiOled()
oled.log("Hello Tiki!")
```

### Tiki API 주요 함수

- **모터 제어**: `forward()`, `backward()`, `clockwise()`, `counter_clockwise()`, `stop()`
- **센서**: `get_battery_voltage()`, `get_current()`, `get_imu()`, `get_encoder()`
- **LED**: `set_led()`, `set_pattern()` (X, O, # 패턴 지원)
- **OLED**: `log()`, `log_clear()`

자세한 내용은 `litebot/io/tiki/` 디렉토리의 각 모듈을 참조하세요.

### Tiki 모드 상세 가이드

**⚠️ 중요**: 실제 하드웨어에서 Tiki 모드를 사용하기 전에 반드시 읽어보세요.

- **[Tiki 모드 사용 가이드](docs/TIKI_MODE_GUIDE.md)**: 캘리브레이션, 설정, 문제 해결 등 상세 정보