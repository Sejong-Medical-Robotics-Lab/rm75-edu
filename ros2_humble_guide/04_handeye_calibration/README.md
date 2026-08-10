# 04. 비전 준비 — 뎁스카메라 테스트부터 Eye-in-Hand 캘리브레이션까지 (Humble)

**목표**
1. RealSense D435 뎁스카메라를 PC에서 인식시키고 정상 동작을 확인한다.
2. 비전 전용 python 가상환경(venv)을 만들고 YOLO로 물체의 **카메라 기준 3D 좌표**를 뽑는다.
3. 팔 말단에 장착된 카메라의 **Eye-in-Hand 캘리브레이션**(`T_ee_cam`)을 수행한다.
4. 캘리브레이션을 **진단 → 검증**하고, 오차가 크면 **무엇을 바꿔야 하는지** 데이터로 판단한다.

여기서 얻는 `T_ee_cam` 행렬이 05(비전 픽앤플레이스)에서 "카메라가 본 물체 좌표"를
"로봇이 갈 수 있는 좌표"로 바꾸는 다리가 된다.

> 📊 **이 문서의 수치는 전부 우리 실험실 RM75 + D435에서 실측한 것이다.**
> 캘리브레이션을 4회 반복하며 오차를 22.5 mm → 6.9 mm로 줄인 과정이 그대로 담겨 있다.
> "왜 가까이서 찍어야 하는지", "왜 자세를 다양하게 해야 하는지"는 의견이 아니라
> 아래 6절의 실측 비교표가 근거다.

> ⚠️ **이 문서는 Jazzy(Ubuntu 24.04) 환경에서 검증한 과정을 Humble(22.04)로 옮긴 것이다.**
> 스크립트 11개는 배포판 의존 코드가 없어 **그대로 동작하며**, pip 패키지의
> Python 3.10 wheel 존재는 확인했다(pyrealsense2 · Robotic_Arm · ultralytics).
> 다만 다음은 Humble 실기에서 미확인이다 — 다르게 나오면 멘토에게 공유한다:
> torch CUDA 빌드 설치, 22.04 + librealsense 조합, 전체 파이프라인 완주.

---

## 0. 워크스페이스 구조 — 왜 `~/ros2_ws`가 아닌가

이 문서부터는 작업 공간이 **둘**이 된다.

| 경로 | 용도 | venv |
|---|---|---|
| `~/ros2_ws` | 01~03의 colcon 워크스페이스 (드라이버 · MoveIt) | 사용 안 함 |
| `~/robot_vision` | **비전 · 캘리브레이션 스크립트** (이 문서) | `vision_env` 사용 |

**분리하는 이유** — ultralytics(YOLO) · torch · pyrealsense2를 시스템 python에 설치하면
ROS 패키지와 의존성이 충돌한다. 그래서 비전 작업은 venv 안에서, ROS 작업은 venv 밖에서 한다.

> ⚠️ **철칙: colcon 빌드와 `ros2 launch`는 반드시 `deactivate` 상태에서.**
> venv가 켜진 채 빌드하면 colcon이 venv의 python을 잡아 깨진다.
> 반대로 이 문서의 python 스크립트는 반드시 venv를 켜고 실행한다.
> 터미널 프롬프트에 `(vision_env)`가 있는지가 구분 기준이다.

두 세계를 **한 터미널에서 같이 써야 할 때**(ROS 토픽을 다루는 python 스크립트)는
ROS를 먼저, venv를 나중에 source 한다:

```bash
source /opt/ros/humble/setup.bash
source ~/robot_vision/vision_env/bin/activate
```

---

## 1. 카메라 인식 확인

카메라(RealSense D435)를 PC USB에 연결한다. **USB 3.0 포트**(파란색)에 꽂을 것 —
2.0에 꽂으면 인식은 되지만 깊이 스트림 해상도가 제한된다.

```bash
lsusb
```

목록에서 카메라를 못 찾겠으면 **USB를 뺀 상태와 꽂은 상태의 `lsusb` 출력을 비교**한다 —
늘어난 줄이 카메라다 (제조사명이 `Intel` 로 안 나오는 경우도 있다).

```bash
sudo apt update && sudo apt install -y v4l-utils
v4l2-ctl --list-devices     # /dev/video* 장치로 잡혔는지
```

마지막으로 공식 뷰어로 영상까지 확인한다:

```bash
realsense-viewer
```

- 좌측에서 **Stereo Module**(깊이)과 **RGB Camera**를 각각 켜서 둘 다 영상이 나오는지 확인
- 깊이 영상에 손을 갖다 대면 색(거리)이 변하는지 확인

> 🔧 `realsense-viewer`가 없다면 librealsense 설치가 필요하다.
> [공식 설치 문서](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)를
> 따르되, 이미 카메라를 써 본 PC(연구용)라면 설치되어 있을 것이다.

---

## 2. 비전 가상환경 구축 + YOLO 3D 좌표 추출

### 2-1. venv 생성과 패키지 설치

```bash
mkdir -p ~/robot_vision
cd ~/robot_vision

sudo apt install -y python3.10-venv   # 22.04의 기본 python은 3.10
python3 -m venv vision_env
source vision_env/bin/activate

pip install --upgrade pip
```

PyTorch는 **자기 PC의 GPU에 맞는 CUDA 빌드**로 설치한다. 먼저 드라이버를 확인:

```bash
nvidia-smi        # 우측 상단 "CUDA Version: 12.x" 를 본다
```

[pytorch.org](https://pytorch.org/get-started/locally/)에서 그 CUDA 버전에 맞는
`--index-url`을 확인해 설치한다. 예:

```bash
# CUDA 12.1 계열 드라이버라면
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
# 최신 GPU(RTX 50 시리즈)라면 cu128 — 선배 PC(Jazzy)에서 검증된 조합
# NVIDIA GPU가 없으면 --index-url 없이 설치 (CPU 동작 — YOLO가 느리지만 실습은 가능)

pip install pyyaml typeguard
```

CUDA가 실제로 잡히는지 확인:

```bash
python3 -c "
import torch
print('torch:', torch.__version__)
print('CUDA available:', torch.cuda.is_available())
print('GPU:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'None')
"
```

`CUDA available: True`가 나와야 한다. False면 NVIDIA 드라이버·CUDA 호환을 먼저 해결한다.

비전 패키지 설치와 임포트 확인:

```bash
pip install ultralytics pyrealsense2 opencv-python numpy

python3 -c "
import cv2
import pyrealsense2 as rs
from ultralytics import YOLO
print('OpenCV:', cv2.__version__)
print('RealSense devices:', len(rs.context().query_devices()))
print('imports OK')
"
```

`RealSense devices: 1`이 나오면 카메라까지 잡힌 것이다. **0이면** USB 연결과 1절을 다시 확인.

### 2-2. 단계별 테스트 — 카메라 → YOLO → 3D 좌표

한 번에 다 돌리지 말고 **세 단계로 나눠** 어디까지 되는지 확인하며 간다.
스크립트는 저장소의 `04_handeye_calibration/scripts/`에 있다 (Jazzy 가이드와 **동일한 파일**이다). `~/robot_vision/`에 복사해 사용한다.

| 스크립트 | 확인하는 것 |
|---|---|
| `test_realsense.py` | RGB + 깊이 스트림이 python에서 열리는가 |
| `test_yolo.py` | YOLO 검출이 RGB 영상에서 도는가 (GPU 사용 확인) |
| `get_3d_coord.py` | 검출 박스 중심의 **카메라 기준 3D 좌표**가 나오는가 |

```bash
cd ~/robot_vision
source vision_env/bin/activate
python3 test_realsense.py     # RGB/깊이 창 확인 후 q로 종료
python3 test_yolo.py          # 검출 박스 확인
python3 get_3d_coord.py       # 물체를 비추면 X, Y, Z (m) 출력
```

`get_3d_coord.py`가 출력하는 좌표는 **camera_color_optical_frame 기준**이다:
X = 오른쪽, Y = 아래, Z = 카메라가 보는 방향(깊이). 물체를 좌우로 옮기며 부호가 맞는지 확인해 두자.

> 💡 `get_3d_coord.py`에는 실전에서 필요한 처리가 이미 들어 있다 —
> 가장 신뢰도 높은 물체 하나만 선택, median smoothing(평균은 순간 튐에 약하다),
> 깊이 유효 범위 제한, 물체가 사라지면 smoothing 초기화.
> 05에서 이 좌표가 로봇 좌표로 변환된다.

---

## 3. Eye-in-Hand 캘리브레이션 — 개념과 준비

### 3-1. 무엇을 구하는 것인가

카메라는 물체를 **자기 기준**으로 본다. 로봇은 **base_link 기준** 좌표로 움직인다.
둘을 잇는 변환 사슬은:

```
base_link ── T_base_ee ──▶ Link7 ── T_ee_cam ──▶ camera ── (YOLO+깊이) ──▶ 물체
             (로봇이 앎)          (❓ 이걸 구한다)
```

`T_base_ee`는 로봇이 항상 알고 있다(정기구학). **모르는 것은 `T_ee_cam` 하나** —
"카메라가 플랜지에서 어디에, 어떤 방향으로 붙어 있는가"다. 이것을 구하는 것이
Eye-in-Hand 캘리브레이션이다.

방법: 고정된 체커보드를 **서로 다른 팔 자세에서 여러 장** 찍는다. 각 장마다
(로봇 자세, 보드가 카메라에 보이는 위치) 쌍이 생기고, "보드는 움직이지 않았다"는
구속으로 `AX = XB` 방정식을 풀면 X = `T_ee_cam`이 나온다.

### 3-2. 준비물

- **체커보드**: 내부 코너 8×5, 한 칸 29 mm (`config.yaml`과 일치해야 함)
  - **평평한 판(아크릴/폼보드)에 부착**할 것 — 종이가 휘면 어떤 방법으로도 오차를 못 줄인다
  - 테이블에 고정한다. **수집 내내 절대 움직이면 안 된다**
- 카메라가 Link7 부근 브래킷에 **단단히** 장착된 상태 (수집 중 미세하게라도 움직이면 무효)
- 로봇 전원 ON, **`rm_driver`는 꺼진 상태** (아래 3-4 참고)

### 3-3. 캘리브레이션 도구 설치

RealMan 공식 캘리브레이션 저장소를 쓴다. 우리 환경에 맞게 몇 가지를 손본다.

```bash
cd ~/robot_vision
source vision_env/bin/activate

git clone https://github.com/RealManRobot/hand_eye_calibration.git
cd hand_eye_calibration
```

```bash
pip install -r requirements.txt
```

> 💡 **Jazzy 가이드에 있는 pyrealsense2 버전 교체(sed)는 Humble에서는 필요 없다.**
> 그 교체는 requirements의 고정 버전(2.55.1.6486)이 **Python 3.12 wheel이 없어서**
> 필요했던 것인데, Humble의 Python 3.10에는 wheel이 존재한다(확인됨).
> 그대로 설치하면 된다. 만약 pyrealsense2 관련 설치 오류가 나면 그때
> `sed -i 's/pyrealsense2==2.55.1.6486/pyrealsense2==2.58.1.10581/' requirements.txt`
> 후 재시도한다 (2.58도 3.10 wheel이 있음을 확인했다).

체커보드 규격을 설정 파일에 반영:

```bash
gedit config.yaml
```

```yaml
checkerboard_args:
  XX: 8        # 내부 코너 가로
  YY: 5        # 내부 코너 세로
  L: 0.029     # 한 칸 크기 (m)
```

> ⚠️ **XX·YY는 칸 수가 아니라 내부 코너 수다.** 8×5 코너 = 9×6 칸.
> 여기가 틀리면 검출 자체가 안 되거나, 되더라도 결과가 엉망이 된다.

### 3-4. 로봇 연결 확인 — `rm_driver`와 동시 사용 금지

수집 스크립트는 ROS를 쓰지 않고 **RealMan SDK로 8080 포트에 직접** 붙는다.
`rm_driver`도 같은 포트를 쓰므로 **둘을 동시에 켜면 안 된다.**

```bash
ros2 node list | grep rm_driver     # 아무것도 안 나와야 함. 나오면 그 터미널을 Ctrl+C
```

연결과 자세 스트림이 안정적인지 먼저 확인한다:

```bash
cd ~/robot_vision/hand_eye_calibration
python3 watch_rm_pose.py            # 말단 XYZ/RPY가 실시간 갱신되면 정상. Ctrl+C로 종료
```

> 💡 수집 로그에 `오류 코드 8193` WARNING이 계속 찍히지만, pose 데이터는 정상적으로
> 오므로 **무시하고 진행해도 된다** (수집 스크립트가 그렇게 처리하도록 되어 있다).

---

## 4. 데이터 수집 — 결과를 좌우하는 단계

**이 절이 캘리브레이션 품질의 전부다.** 계산은 어느 방법을 써도 비슷하게 나온다(6절 실측 참고).
차이를 만드는 것은 데이터다.

### 4-1. 수집 규칙 (실측으로 확정된 것)

| 규칙 | 값 | 근거 (6절 비교표) |
|---|---|---|
| **촬영 거리** | **0.4 ~ 0.9 m** (보드가 화면의 절반 이상) | 1.0 m 이상으로 찍은 2트: 22.5 mm → 거리 줄인 3트: 10.9 mm |
| **자세 다양성** | 카메라가 보드를 보는 **기울기 자체**를 크게 바꿈 | 기울기가 한 범위에 몰린 3트: 10.9 mm → 다양화한 4트: **6.9 mm** |
| **장수** | 20~26장 | 많이 찍는 것보다 **다양하게** 찍는 것이 중요 |
| **정지 후 촬영** | 드래그 티칭에서 손을 뗀 뒤 **2~3초 기다렸다가** 저장 | 팔이 완전히 멈추기 전 저장하면 pose와 이미지가 어긋남 |

자세 다양성을 구체적으로:

- 보드를 **위에서 내려다보는** 자세만 반복하지 않는다 (가장 흔한 실수)
- **거의 정면**에서 보는 자세, **아래에서 올려다보는** 자세, **좌우 비스듬한** 자세를 섞는다
- 카메라 **광축 회전(화면이 기울어지는 방향)** 도 여러 값으로
- 매 장이 직전 장과 **눈에 띄게 달라야** 한다 — 비슷한 자세 두 장은 정보가 한 장 값어치다

### 4-2. 수집 실행

```bash
cd ~/robot_vision/hand_eye_calibration
source ~/robot_vision/vision_env/bin/activate
python3 collect_data.py
```

- 시작하면 `eye_hand_data/data<날짜>` 폴더가 만들어지고 카메라 창이 뜬다
- 화면에 체커보드 코너가 색색으로 그려지면(검출 성공) 그 상태에서만 **`s`** 로 저장된다
- 팔을 옮기는 방법: **드래그 티칭** — 팔의 초록 버튼을 누른 채 손으로 이끌고, 놓고, 2~3초 후 `s`
- 매 저장마다 터미널에 `N번째 데이터 수집 완료`와 로봇 자세 6값이 찍힌다
- 목표 장수를 채우면 **`q`** 로 종료

> 💡 수집 중 터미널의 로봇 자세 `[x, y, z, rx, ry, rz]`에서 **다섯 번째 값(ry)** 을 흘끗 보자.
> 이 값이 내내 비슷하면(예: 계속 0.9~1.5) 기울기 다양성이 부족하다는 신호다 —
> 0.5 이하, 1.6 이상인 자세를 의식적으로 추가한다.

---

## 5. 진단 → 계산

### 5-1. 계산 전에 반드시 진단부터

```bash
cd ~/robot_vision/hand_eye_calibration
source ~/robot_vision/vision_env/bin/activate
python3 diagnose_calibration.py
```

세 가지를 검사한다. **각각이 서로 다른 실패 원인을 가려낸다.**

| 검사 | 정상 기준 | 나쁘면 의심할 것 |
|---|---|---|
| ① 이미지별 재투영 오차 | **0.5 px 이하** | 카메라 내부 파라미터 · 체커보드 규격 설정 · 초점 |
| ② 4가지 계산 방법(TSAI/PARK/HORAUD/DANIILIDIS) 비교 | 방법 간 산포 **수 mm** | 데이터 자체의 모순 (극단적 이상치) |
| ③ 이미지별 체커보드 위치 일관성 | 평균 오차 **10 mm 안팎**, `<-- 의심` 없음 | **촬영 거리 · 자세 다양성 · 정지 전 촬영** |

읽는 법: ①②가 좋고 ③만 나쁘면 **계산이 아니라 데이터 수집이 문제**다 — 4-1 규칙 중
무엇을 어겼는지 찾아 재수집한다. `<-- 의심` 표시가 붙은 이미지가 몇 장뿐이면
그 파일만 지우고 다시 진단해 봐도 된다.

### 5-2. 계산

진단이 통과하면 계산한다. **결과는 회차별 파일로 남긴다** (나중에 비교하기 위해):

```bash
python3 compute_in_hand.py 2>&1 | tee calibration_result_$(date +%m%d).txt

grep -n "회전행렬" calibration_result_*.txt    # 내용이 들어갔는지 확인
```

> ⚠️ **`2>&1`을 빼면 파일이 비어서 저장된다.** `compute_in_hand.py`는 결과를 로거로
> 출력하는데 로그는 **stderr**로 나가고 `| tee`는 stdout만 잡기 때문이다.
> 화면에는 멀쩡히 보이는데 파일만 비는 형태라 알아채기 어렵다 —
> 저장 직후 위 `grep`으로 반드시 확인할 것.

출력에서 쓸 것은 셋이다:

```
회전행렬:  3×3
병진벡터:  [x, y, z]  (m)
사원수:    [qx, qy, qz, qw]
```

**타당성 즉석 점검** — 자로 잰 카메라 장착 위치와 병진벡터가 대략(±2 cm) 맞는지,
회전이 장착 방향과 상식적으로 맞는지 눈으로 확인한다. 예를 들어 우리 장착에서는
z축 −90° 회전에 가까운 값이 나와야 한다. 수십 cm씩 다르면 어딘가 크게 잘못된 것이다.

---

## 6. 검증 — TF로 실측하기

계산된 `T_ee_cam`이 실제로 맞는지, **"고정된 체커보드는 팔을 어떻게 움직여도
base_link 기준 같은 위치여야 한다"** 는 성질로 검증한다.

### 6-1. 원리

```
base_link → Link7        : rm_tf_publisher.py  (SDK로 로봇 자세를 읽어 TF 발행)
Link7 → camera_..._frame : static TF           (5-2에서 계산한 T_ee_cam)
camera → checkerboard    : checkerboard_tf_publisher.py  (실시간 보드 검출)
```

세 변환을 이으면 `base_link → checkerboard`가 나온다. 팔을 이리저리 옮기며
이 값을 여러 번 측정했을 때 **흔들림(산포)이 곧 캘리브레이션 오차**다.

### 6-2. static TF 명령 만들기

```bash
python3 make_tf_cmd.py
```

인자 없이 실행하면 **가장 최근 `calibration_result_*.txt`를 자동으로 읽는다.**
특정 파일을 쓰려면 `python3 make_tf_cmd.py calibration_result_0810.txt`.

출력은 세 부분이다:

```
[타당성 점검]
  기대 장착 자세(z축 -90도)와의 차이: 1.32 deg
  카메라까지의 거리: 106.5 mm
  이상 없음

[static TF 명령] ...          ← 복사해서 터미널2에 붙여넣는다
[05에서 쓸 값] ...            ← 05에서 T_ee_cam으로 쓴다
```

**타당성 점검을 먼저 읽자.** 기대 장착 자세와의 차이가 5도를 넘거나 카메라 거리가
자로 잰 값과 크게 다르면 경고가 뜬다 — 그 상태로 검증해 봐야 시간만 버린다.

> 💡 이 스크립트는 결과 파일을 직접 읽으므로 **값을 손으로 옮겨 적지 않는다.**
> 원본(RealMan 저장소)은 행렬이 하드코딩되어 있어 재계산 후 갱신을 잊고
> **옛 값으로 검증하는 사고**가 실제로 있었다. 그래도 검증 결과가 이상하게 나쁘면
> 터미널2의 `--x` 값과 `[05에서 쓸 값]`의 첫 숫자가 같은지 확인하라.
>
> 장착 방향이 다른 카메라를 쓴다면 스크립트 상단의 `EXPECTED_ROTATION`을 수정한다.

### 6-3. 4터미널 검증 실행

체커보드가 카메라에 보이는 위치에 놓인 상태에서, 터미널 **넷을 전부 켜 둔 채** 진행한다.
하나라도 끄면 TF 사슬이 끊긴다.

```bash
# 터미널1 — 로봇 TF (base_link → Link7)
source /opt/ros/humble/setup.bash
source ~/robot_vision/vision_env/bin/activate
cd ~/robot_vision && python3 rm_tf_publisher.py
```

```bash
# 터미널2 — 캘리브레이션 static TF (6-2에서 복사한 명령. venv 불필요)
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher \
  --x <값> --y <값> --z <값> --qx <값> --qy <값> --qz <값> --qw <값> \
  --frame-id Link7 --child-frame-id camera_color_optical_frame
```

```bash
# 터미널3 — 체커보드 TF (창에 DETECTED(초록)가 떠야 함)
source /opt/ros/humble/setup.bash
source ~/robot_vision/vision_env/bin/activate
cd ~/robot_vision/hand_eye_calibration && python3 checkerboard_tf_publisher.py
```

```bash
# 터미널4 — 검증
source /opt/ros/humble/setup.bash
source ~/robot_vision/vision_env/bin/activate
cd ~/robot_vision/hand_eye_calibration && python3 verify_handeye.py
```

TF 사슬이 이어졌는지 먼저 확인하고 싶으면 터미널을 하나 더 열어:

```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros tf2_echo base_link checkerboard    # 값이 나오면 사슬 완성
```

**측정 방법** — 터미널4에서, 팔을 드래그 티칭으로 **크게 다른 자세**로 옮기고 →
**2~3초 정지** → Enter. 8~10회 반복 후 `q`.

> 💡 결과는 화면에 출력되는 동시에 **`verify_result_<날짜시각>.txt`로 저장**된다 —
> 회차별로 비교할 때 쓴다 (6-5 참고).
>
> 원본 스크립트는 종료 시 `terminate called without an active exception /
> Aborted (core dumped)`를 뱉었다(결과에는 영향 없음). 저장소의 수정본은
> spin 스레드를 정리하도록 고쳐 이 메시지가 나오지 않는다.

### 6-4. 판정

```
축별 산포: X=.. Y=.. Z=.. mm
평균 오차: .. mm
```

| 평균 오차 | 판정 |
|---|---|
| 5 mm 이내 | 매우 좋음 — 그대로 사용 |
| 5~15 mm | **양호, 실용 가능** — 05로 진행 |
| 15~30 mm | 재수집 권장 — 아래 실측 사례로 원인 판단 |
| 30 mm 이상 | 어딘가 잘못됨 — static TF 값·체커보드 고정·카메라 장착부터 의심 |
| 특정 자세만 10~20 cm 점프 | 체커보드 180° 모호성 — 그 측정만 버린다 |

### 6-5. 실측 사례 — 오차를 22.5 → 6.9 mm로 줄인 과정

우리 실험실에서 실제로 밟은 과정이다 (측정은 선배의 Jazzy PC에서 했지만,
오차를 만드는 요인은 카메라·로봇·체커보드 쪽이라 **판단 기준은 배포판과 무관**하다). **오차가 15 mm를 넘으면 이 표에서 자기 상황을 찾아라.**

| 회차 | 바꾼 것 | 촬영 거리 | 평균 오차 | 축별 산포 (X/Y/Z mm) |
|---|---|---|---|---|
| 2트 | — (기준) | 0.75~1.37 m | 22.5 mm | 42.9 / 38.6 / **60.7** |
| 3트 | **거리 단축** | 0.50~0.90 m | 10.9 mm | 24.5 / 15.0 / **25.8** |
| 4트 | **+ 자세(기울기) 다양화** | 0.39~0.88 m | **6.9 mm** | 14.5 / 9.9 / 15.6 |

읽는 법:

- **Z 산포가 유독 크면 → 너무 멀리서 찍었다.** Z는 카메라 광축 방향이라 거리 오차가
  그대로 실린다. 2트→3트에서 거리만 줄였는데 Z 산포가 60.7 → 25.8로 반토막 났다.
- **거리를 줄였는데도 10 mm 위에 머물면 → 자세 다양성 부족.** 3트→4트에서 보드를 보는
  기울기(정면/위/아래/비스듬)를 다양화하자 6.9 mm까지 내려갔다. 회전 추정이 좋아진 것은
  별도 지표로도 확인된다 — 계산된 회전과 장착 설계상 이상 회전(z축 −90°)의 차이가
  2트 6.4° → 3트 2.1° → 4트 **1.3°** 로 수렴했다.
- **4트 수준(±7 mm)이 이 장비 구성의 실질 한계로 보인다.** 그 아래는 체커보드 평탄도,
  드래그 티칭 정지 정밀도, 로봇 기구학 오차의 영역이라 수집을 더 잘해도 내려가지 않는다.

### 6-6. (선택) RViz 시각 검증

숫자가 아니라 눈으로 보고 싶으면: `rm_joint_publisher.py`(관절각 발행) +
`robot_state_publisher`(01의 `rm_description` 사용) + static TF + 체커보드 TF + RViz를 띄우면
로봇 모델 위에 카메라·체커보드 프레임이 그려진다. 팔을 움직여도 체커보드 프레임이
제자리에 있으면 성공이다. 상세 절차는 `04_handeye_calibration/scripts/README`를 참고.

---

## 7. 산출물 — 05로 가져갈 것

- **`T_ee_cam`**: 병진 3값 + 사원수 4값 (또는 static TF 명령 한 줄)
- 검증 수치: 평균 오차, 축별 산포 — **05에서 파지 여유를 정할 때 근거가 된다**
  (예: 오차 ±7 mm면 그리퍼 개방 여유가 그보다 커야 안정적으로 잡힌다)
- `calibration_result_*.txt` 원본과 데이터 폴더 — 재현·비교용으로 보존

> 우리 실험실 4트 최종값 (2026-08-10, 검증 평균 오차 6.9 mm):
>
> ```bash
> ros2 run tf2_ros static_transform_publisher \
>   --x -0.092640 --y 0.048367 --z -0.020571 \
>   --qx 0.001056 --qy 0.003856 --qz -0.699441 --qw 0.714679 \
>   --frame-id Link7 --child-frame-id camera_color_optical_frame
> ```
>
> ⚠️ **카메라를 떼었다 다시 달면 이 값은 무효다** — 재캘리브레이션해야 한다.
> 브래킷을 만진 적이 있는지 불확실하면 6-3 검증만 다시 돌려 오차를 확인하라.

---

## 트러블슈팅

| 증상 | 확인할 것 |
|---|---|
| `RealSense devices: 0` | USB 재연결 · 3.0 포트인지 · `realsense-viewer`로 원점 확인 |
| `pip install -r requirements.txt` 충돌 | pyrealsense2 관련이면 3-3의 참고(버전 교체 sed) 시도 |
| `... compiled using NumPy 1.x cannot be run in NumPy 2.x ...` | `pip install "numpy<2"` 후 재시도 (Humble 계열의 알려진 조합 문제) |
| `make_tf_cmd.py`가 "회전행렬을 찾지 못했습니다" | 결과 파일이 비었다 — `2>&1`을 빼고 저장했을 가능성 (5-2) |
| `make_tf_cmd.py`가 옛 값을 출력함 | 스크립트를 새 버전으로 교체했는지 (`[타당성 점검]` 문구가 나오면 새 버전) |
| colcon 빌드가 이상하게 깨짐 | **venv가 켜진 채 빌드했는지** — `deactivate` 후 재시도 |
| `collect_data.py` 로봇 연결 실패 | `rm_driver`가 떠 있는지 (`ros2 node list`) · 로봇 전원 · ping |
| 수집 로그에 `오류 코드 8193` | 정상 — pose는 오고 있으므로 무시 |
| 체커보드 검출이 안 됨 (`s`가 안 먹힘) | `config.yaml`의 XX/YY가 **내부 코너 수**인지 · 조명 반사 · 거리 |
| 진단 ③ 평균 오차 15 mm↑ | **6-5 실측 사례 표** — 거리부터, 그다음 자세 다양성 |
| 검증 오차가 진단보다 훨씬 나쁨 | static TF에 **옛 값**이 들어갔는지 (6-2 경고) · 체커보드가 수집 후 움직였는지 |
| `verify_handeye.py` 종료 시 `Aborted (core dumped)` | 정상 — 결과 출력 후에 나는 것이므로 무시 |
| TF 나이가 수백 ms 이상 | `rm_tf_publisher.py` 터미널이 살아 있는지 · 로봇 통신 상태 |

캘리브레이션이 끝났으면 → [05_vision_grasp](../05_vision_grasp/README.md) *(작성 예정)*
