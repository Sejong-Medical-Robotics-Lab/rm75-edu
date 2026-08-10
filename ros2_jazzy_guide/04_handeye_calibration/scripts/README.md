# 04_handeye_calibration / scripts

04 문서에서 사용하는 스크립트 모음이다.
**모두 `~/robot_vision/` 아래에 복사해 사용한다** (colcon 패키지가 아니다).

## 설치 위치

| 스크립트 | 복사할 위치 | 실행 환경 |
|---|---|---|
| `test_realsense.py` | `~/robot_vision/` | venv |
| `test_yolo.py` | `~/robot_vision/` | venv |
| `get_3d_coord.py` | `~/robot_vision/` | venv |
| `rm_tf_publisher.py` | `~/robot_vision/` | ROS + venv |
| `rm_joint_publisher.py` | `~/robot_vision/` | ROS + venv |
| `watch_rm_pose.py` | `~/robot_vision/hand_eye_calibration/` | venv |
| `collect_data.py` | `~/robot_vision/hand_eye_calibration/` ⚠️ **덮어쓰기** | venv |
| `diagnose_calibration.py` | `~/robot_vision/hand_eye_calibration/` | venv |
| `make_tf_cmd.py` | `~/robot_vision/hand_eye_calibration/` | venv |
| `checkerboard_tf_publisher.py` | `~/robot_vision/hand_eye_calibration/` | ROS + venv |
| `verify_handeye.py` | `~/robot_vision/hand_eye_calibration/` | ROS + venv |

```bash
# 04 문서 3-3에서 hand_eye_calibration을 clone한 뒤
cd ~/rm75-edu/ros2_jazzy_guide/04_handeye_calibration/scripts

cp test_realsense.py test_yolo.py get_3d_coord.py \
   rm_tf_publisher.py rm_joint_publisher.py ~/robot_vision/

cp watch_rm_pose.py collect_data.py diagnose_calibration.py \
   make_tf_cmd.py checkerboard_tf_publisher.py verify_handeye.py \
   ~/robot_vision/hand_eye_calibration/
```

> ⚠️ **`collect_data.py`는 원본을 덮어쓴다.** 원본은 체커보드 검출 여부를 화면에
> 표시하지 않아, 검출이 안 된 상태에서 저장해 데이터가 오염되기 쉽다.
> 덮어쓰기 전 원본을 백업해 두려면: `cp collect_data.py collect_data.py.orig`

## 실행 환경 구분

| 표기 | 의미 |
|---|---|
| **venv** | `source ~/robot_vision/vision_env/bin/activate` 만 |
| **ROS + venv** | `source /opt/ros/jazzy/setup.bash` **먼저**, 그다음 venv |

순서를 바꾸면 rclpy 임포트가 깨진다.

---

## 원본 대비 수정 사항

RealMan 공식 저장소(`RealManRobot/hand_eye_calibration`)의 스크립트를 우리 환경에
맞게 고쳤다. 파이퍼(Piper) 기준 예제를 RM75로 옮기며 바뀐 점:

```
CAN 통신          →  Ethernet 통신 (8080 포트, JSON)
piper_ws 드라이버  →  RealMan SDK (Robotic_Arm)
Piper URDF        →  RM75 URDF
6자유도           →  7자유도
```

### `collect_data.py`

- **체커보드 검출 상태를 화면에 표시** (`CHECKERBOARD DETECTED` / `NOT DETECTED`)
- **검출된 경우에만 `s` 저장 허용** — 검출 실패 상태로 저장되던 문제 해결
- 저장된 개수를 화면에 표시, `q`로 정상 종료
- 화면 표시만 2배 확대 (저장 이미지는 원본 640×480 유지)
- **오류 코드 8193 무시** — 그리퍼 관련 오류가 상시 발생하지만 pose는 정상이므로
  캘리브레이션 수집을 중단시키지 않는다
- pose 저장 실패 시 이미지도 삭제 — 이미지와 pose 개수가 어긋나는 것을 방지

### `make_tf_cmd.py`

- **결과 파일(`calibration_result_*.txt`)을 직접 읽는다.** 원본은 회전행렬·병진벡터가
  하드코딩되어 있어, 재계산 후 갱신을 잊고 **옛 값으로 검증하는 사고**가 실제로 있었다
- 인자 없이 실행하면 가장 최근 결과 파일을 자동 선택
- **타당성 자동 점검** — 정규직교성, 기대 장착 자세와의 각도 차이, 카메라까지의 거리
- 05에서 쓸 값(translation, quaternion)도 함께 출력

### `verify_handeye.py`

- **종료 시 `Aborted (core dumped)` 해결** — executor 종료 후 spin 스레드를 `join`
  (결과 출력에는 영향이 없었지만 실패로 오해하기 쉬웠다)
- TF 나이 검사 — 오래된 값을 잘못 기록하는 것 방지
- 직전과 동일한 값이면 거부 — TF가 갱신되지 않은 상태 감지
- **결과를 `verify_result_<날짜시각>.txt`로 저장** — 회차별 비교용

### `diagnose_calibration.py`

원본에 없던 스크립트. 계산 전에 데이터 품질을 세 갈래로 진단한다
(재투영 오차 / 방법 간 일관성 / 이미지별 보드 위치). 04 문서 5-1 참고.

---

## 05에서 이어 쓰는 것

- `get_3d_coord.py` — 카메라 기준 3D 좌표 추출 로직 (median smoothing 포함)
- `make_tf_cmd.py` 출력의 translation·quaternion — `T_ee_cam` 행렬
- `rm_tf_publisher.py` — 로봇 TF 발행 (MoveIt bringup을 쓰면 불필요)
