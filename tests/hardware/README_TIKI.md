# Tiki 모드 테스트 가이드

## ROS → Tiki 변환 체크리스트

ROS 테스트를 Tiki 모드로 변환할 때 다음 사항들을 확인하세요:

### 1. Import 변경
```python
# ROS
from litebot.io.ros.ros_controller import ROSController
from litebot.io.ros.ros_camera import ROSCamera

# Tiki
from litebot.io.tiki.tiki_controller import TikiController
from litebot.io.tiki.tiki_camera import TikiCamera
```

### 2. ROS 초기화 제거
```python
# ROS (제거 필요)
rospy.init_node("node_name", anonymous=False)

# Tiki (불필요)
# ROS 초기화 코드 제거
```

### 3. 시간 관련 함수 변경
```python
# ROS
rospy.sleep(0.5)
rate = rospy.Rate(20)
rate.sleep()
start = rospy.Time.now()
duration = (rospy.Time.now() - start).to_sec()

# Tiki
import time
time.sleep(0.5)
time.sleep(0.05)  # 20Hz = 0.05초
start = time.time()
duration = time.time() - start
```

### 4. ROS 체크 제거
```python
# ROS (제거 필요)
if rospy is None:
    print("rospy unavailable")
    sys.exit(1)

# Tiki (불필요)
# ROS 체크 코드 제거
```

### 5. 클래스 인스턴스화
```python
# ROS
ctrl = ROSController()
cam = ROSCamera()

# Tiki
ctrl = TikiController()
cam = TikiCamera()
```

## 이미 존재하는 Tiki 테스트 파일

다음 테스트 파일들은 이미 Tiki 모드로 구현되어 있습니다:

- `tiki_controller_run.py` - 컨트롤러 단독 테스트
- `tiki_camera_run.py` - 카메라 테스트
- `tiki_executor_run.py` - ActionExecutor 테스트
- `tiki_resource_separation.py` - 리소스 분리 테스트

## 실행 방법

```bash
# Tiki 컨트롤러 테스트
python tests/hardware/tiki_controller_run.py

# Tiki 카메라 테스트
python tests/hardware/tiki_camera_run.py

# Tiki Executor 테스트
python tests/hardware/tiki_executor_run.py

# Tiki 리소스 분리 테스트
python tests/hardware/tiki_resource_separation.py
```

## 주의사항

1. **Tiki 라이브러리 필요**: `tiki.mini` 패키지가 설치되어 있어야 합니다.
2. **하드웨어 연결**: 실제 Tiki 로봇이 연결되어 있어야 정상 동작합니다.
3. **Stub 모드**: Tiki 라이브러리가 없으면 stub 모드로 동작하지만 실제 하드웨어 제어는 안 됩니다.

