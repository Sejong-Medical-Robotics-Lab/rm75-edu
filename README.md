# RM75 교육용 저장소 (rm75-edu)

세종대 로봇 플랫폼 교육 프로그램 — **RM75 매니퓰레이터 로테이션 주간(교재 ⑤)**
실습 저장소입니다. MoveIt2 + RViz **demo 모드**(물리 없는 운동학 시뮬레이션,
교재 4.3) 위에서 교재의 토픽 조사·마커 플래닝·코드 예제·그리퍼·pick-and-place
미션을 모두 지원합니다.

> ⚠ **시뮬레이션 전용입니다.** 실기체 연결·조작은 금요일에 멘토 동석
> 하에서만 합니다(교재 2.5·6.1). 그리고 교재 4.9 미션 ①: **이 README를 먼저
> 끝까지 통독한 뒤** 설치를 시작하세요 — 읽으며 따라 하지 말 것.

---

## 1. 교재 ↔ 저장소 대응표

| 교재 위치 | 할 일 | 저장소에서 쓰는 것 |
|---|---|---|
| 3.5~3.6 미션 A | 클론 → colcon build → demo → 토픽·액션 조사 | §3 설치, §6 표(정답지) |
| 4.4 환경 준비 | `ros-humble-moveit` 설치 + demo 실행 | §3·§4 |
| 4.5~4.7 | 마커 독법·체크포인트 ①~⑦·플래너 무작위성 | RViz MotionPlanning 패널 |
| 4.6 체크포인트 ⑦ | 미션 테이블·A·B를 Scene에 미리 | `mission_scene` (§7) |
| 5.4~5.6 미션 B | "홈 → 목표 → 홈" 예제 해부 | `move_home_target` (§7) |
| 5.3 | 그리퍼 세 숫자(폭·속도·힘) | `gripper_demo`, `GripperClient` |
| 7장 미션 | pick-and-place 시퀀스 | `pick_place_template` + `mission_scene` |
| 7.6 인수인계 | HANDOVER.md 3항목 | `HANDOVER_TEMPLATE.md` |
| 부록 B | 빈출 트러블 | §10 트러블슈팅 |

## 2. 요구 환경

Ubuntu 22.04 + ROS 2 Humble(교재 ① 3장에서 준비한 환경). demo 모드는 물리
계산이 없어 가볍습니다 — WSL2에서도 무난합니다(교재 4.4).

## 3. 설치 (교재 3.6 미션 ① / 4.4 — 화요일까지)

```bash
# 0) 클론 (조 공지의 저장소 주소)
cd ~/robot_ws/src
git clone <저장소 주소>          # → ~/robot_ws/src/rm75-edu

# 1) MoveIt2 및 관련 패키지 (교재 4.4의 그 명령)
sudo apt update
sudo apt install -y ros-humble-moveit ros-humble-ros2-control \
                    ros-humble-ros2-controllers ros-humble-xacro

# 2) 빌드 및 환경 적용 — 수요일까지 필요한 3개 패키지
cd ~/robot_ws
colcon build --packages-select rm_description rm75_edu_moveit_config rm75_edu
source install/setup.bash        # 새 터미널마다! (또는 ~/.bashrc 에 추가)
```

저장소에는 제조사 드라이버(`rm_driver`·`rm_ros_interfaces`)도 포함되어
있습니다(교재 3.1) — **금요일 실기체 전용**이며 멘토가 다룹니다. 전체 빌드
(`colcon build`)는 금요일 준비 때 멘토와 함께 합니다.

설치 확인(교재 4.4 확인 기준 세 가지):

```bash
ros2 launch rm75_edu_moveit_config demo.launch.py
# ① RViz 창에 RM75 모델(+그리퍼)이 보인다
# ② 손끝에 주황색 interactive marker(공·화살표·고리)가 보인다
# ③ 터미널에 빨간 오류가 반복 출력되지 않는다
```

## 4. 빠른 시작

```bash
ros2 launch rm75_edu_moveit_config demo.launch.py    # MoveIt2 + RViz (demo)
```

- 로봇은 **home 자세**(테이블 앞을 내려보는 준비 자세)로 시작합니다.
- 마커 조작: 공을 끌면 위치, 고리를 돌리면 자세(교재 4.5). **주황색은 아직
  소원입니다** — Plan(계산) → 잔상 확인 → Execute(실행)의 세 단계를 수요일
  내내 몸에 붙이세요.
- 마커가 안 보이면 §10의 표(교재 4.4 표와 동일) 순서로 점검.

## 5. 플래닝 그룹과 자세 (교재 4.2 "그룹 = 명령의 수신 단위")

| 그룹 | 관절 | named 자세 |
|---|---|---|
| `rm_group` (팔) | joint1~joint7, 손끝 기준점 = `gripper_tcp` | `home`(준비), `zero`(직립) |
| `gripper` | gripper_finger1/2_joint (병렬 2지) | `open`(0.08 m), `closed`(0) |

- pose goal 의 (x, y, z)는 **base_link 좌표계·미터 단위**, 기준점은
  **손끝(gripper_tcp)** 입니다 — "테이블 위 그 지점"을 그대로 넣으면 됩니다
  (단위 실수 주의: 412 mm → 0.412 m, 교재 5.5).
- 자세는 새로 만들지 말고 검증된 **"아래 보기" 상수 `DOWN_Q`**
  (`rm75_edu/poses.py`)를 재사용하세요(교재 5.2). home 자세의 손끝이 이미
  이 자세입니다.

## 6. 토픽·액션 표 (미션 A의 정답지)

이름은 외우지 말고 `ros2 topic list` / `ros2 action list` 로 직접 확인하는
것이 원칙(교재 3.2) — 아래는 조사 결과를 채점할 때의 정답지입니다.
주기(Hz)는 표에 적지 않았습니다: `ros2 topic hz` 로 **실측해 기록**하는
것까지가 미션입니다.

| 데이터 (교재 3.2) | 실제 이름 | 타입/방식 |
|---|---|---|
| 관절 상태 | `/joint_states` — 팔 7 + 그리퍼 2 = **9관절** | `sensor_msgs/JointState` 토픽 |
| 궤적 명령(팔) | `/rm_group_controller/follow_joint_trajectory` | **액션** (`control_msgs/FollowJointTrajectory`) |
| 그리퍼 명령 | `/gripper_controller/follow_joint_trajectory` | **액션** (상태는 `/joint_states` 로) |
| 플래닝 요청/실행 | `/move_action` · `/execute_trajectory` | **액션** (`moveit_msgs`) — ArmClient 가 쓰는 통로 |
| 모드/에러 상태 | **없음** — 미션 ③ 표에 "없음"이라 기록 (demo 와 실기체의 차이 자체가 관찰) | — |

- `move_group` 노드가 MoveIt2의 본체입니다 — `ros2 node info /move_group` 과
  rqt_graph 로 "요청이 들어가는 길"과 "궤적이 나가는 길"을 찾는 것이
  미션 ②·⑤.
- effort 관찰(3장 도전 미션): demo 모드의 effort 는 **0으로 채워져**
  나옵니다 — 가상 컨트롤러에는 전류가 없기 때문. 금요일 실기체에서 이 열이
  어떻게 달라지는지가 숨은 관찰 과제입니다.

## 7. 예제 4종 (5·7장)

```bash
# 터미널 1 은 항상 demo 런치가 떠 있는 상태에서:
ros2 run rm75_edu move_home_target     # 미션 B: 홈→목표→홈 (5.4 뼈대 ①~⑦)
ros2 run rm75_edu gripper_demo         # 그리퍼 세 숫자 (5.3)
ros2 run rm75_edu mission_scene        # 미션 Scene: 테이블·블록A·B표식 (4.6⑦/7.2)
ros2 run rm75_edu pick_place_template  # 7.2 시퀀스 표 실행기 (①구간만 예시)
```

- `move_home_target` 은 교재 5.4 뼈대의 ①~⑦ 번호가 코드 주석에 그대로
  달려 있습니다 — 미션 B ①(예제 해부)의 대상이 이 파일입니다. **plan 성공
  확인 없이 execute 하지 않는 구조**(⑤)를 코드에서 찾아보세요.
- `ArmClient` 는 내부적으로 `/move_action`(plan_only=True: 계산만)과
  `/execute_trajectory`(실행)를 나눠 부릅니다 — RViz의 Plan/Execute 버튼과
  같은 분리가 코드에도 있는 셈입니다.
- 그리퍼 `grasp()` 는 demo 모드에서 **항상 성공(흉내)** 입니다 — 물리가
  없기 때문(교재 7.2가 미리 말한 한계). 그래도 반환값을 확인하는 습관을
  들이세요: 실기체에서 그 확인을 빼먹으면 "허공 배달"(5.3)이 됩니다.
- `pick_place_template` 은 ①구간까지만 채워져 있습니다 — ②~⑤는 여러분의
  설계 시트(7.2)를 옮기는 자리입니다. **Scene 을 먼저 등록**(`mission_scene`)
  해야 경로가 테이블·블록을 피해 계획됩니다.

## 8. 관찰 노트

- home 자세의 손끝은 (0.35, 0, 0.02) 부근 — 이미 테이블을 내려보고
  있습니다. 예제 목표 (0.32, 0, 0.25)와 미션 좌표 A/B는 도달성이 사전
  검증된 값입니다(`tools/fk_check.py` — 좌표를 바꾸면 이 도구로 재확인).
- 같은 목표를 5회 Plan 하면 경로가 매번 다를 수 있습니다(OMPL 무작위 탐색,
  교재 4.7) — 버그가 아니라 알고리즘의 성격이며, 그래서 실기체에서 잔상
  확인을 생략할 수 없습니다.
- Plan 실패의 세 갈래(도달·자세·충돌, 교재 4.8)를 일부러 만들어 보세요:
  x=1.5(도달 밖), 무리한 orientation(자세), Scene 블록 관통 경로(충돌).
- Scene 은 "플래너가 아는 세계"입니다 — `mission_scene` 실행 전후로 같은
  목표를 Plan 해 경로가 달라지는 것(체크포인트 ⑥)을 비교 캡처하세요.

## 9. 저장소 구조

```
rm75-edu/
├── rm_description/           # RealMan RM75 URDF·메시 (rm_75만 슬림 발췌)
├── rm_ros_interfaces/        # 제조사 메시지 정의 (실기체용)
├── rm_driver/                # 제조사 ROS2 드라이버 — 금요일 멘토 전용
├── rm75_edu_moveit_config/   # MoveIt2 설정 (그리퍼 포함) + demo.launch.py
├── rm75_edu/                 # ArmClient·GripperClient·예제 4종
├── tools/fk_check.py         # 오프라인 FK/IK 도달성 검증기
├── docs/                     # joint_map · real_robot(멘토) · textbook_notes
└── HANDOVER_TEMPLATE.md      # 인수인계 3항목 (교재 7.6)
```

## 10. 트러블슈팅 (교재 4.4 표·부록 B 대응)

| 증상 | 확인/해결 |
|---|---|
| launch 파일을 못 찾음 | colcon build 후 **모든 터미널**에서 `source ~/robot_ws/install/setup.bash` |
| RViz에 로봇이 안 보임 | Fixed Frame = `base_link` · Displays에 MotionPlanning 추가/체크 |
| 주황색 마커가 없음 | MotionPlanning → Planning Request → Query Goal State 체크 · Planning Group = `rm_group` |
| Plan이 계속 실패 | 부록 B 세 갈래: ① 도달(팔 길이 안? 작업 반경 610 mm) ② 자세(`DOWN_Q` 상수를 쓰는지) ③ 충돌(Scene의 테이블·블록과 겹치는지). 목표를 로봇 가까이로 옮겨 성공부터 재현 |
| 그리퍼가 안 움직임 | demo 런치가 떠 있는지 · `ros2 action list` 에 gripper_controller 액션이 보이는지 |
| RViz가 검게/느리게 뜸 | GPU(nvidia-smi) · WSL2 그래픽(Week 1 FAQ) — demo는 가벼우므로 그래도 느리면 환경 문제 |
| 빌드 오류 | §3의 apt 목록을 빠짐없이 설치했는지 · 오류 첫 줄을 grep으로 찾아 읽기 |
| demo와 실기체가 다름 | 정상 — sim-to-real(부록 B). demo가 못 보는 것: 물체 무게·마찰, 통신 지연, 사람 측정 오차 |

## 11. 라이선스

자체 코드(MoveIt 설정·예제·도구)는 MIT(`LICENSE`).
제조사 패키지(`rm_description`·`rm_ros_interfaces`·`rm_driver`)는
RealMan ros2_rm_robot 발췌 — **`THIRD_PARTY_NOTICES.md` 를 반드시 확인**
(업스트림 라이선스 표기 관련 주의사항 포함).
