# RM75 로봇팔 실습 가이드 (Ubuntu 22.04 + ROS2 Humble)

연구실 신입생을 위한 RealMan RM75 매니퓰레이터 실습 가이드입니다.
[RealMan 공식 저장소](https://github.com/RealManRobot/ros2_rm_robot)의 **humble 브랜치**를 기준으로 하며,
**시뮬레이션 검증 → 실기체 실행** 순서로 진행합니다.

## 대상 환경

| 항목 | 값 |
|---|---|
| OS | Ubuntu 22.04 |
| ROS2 | Humble |
| 로봇 | RealMan RM75 (7자유도 협동 로봇 팔, RM75-BI / 컨트롤러 v3) |
| 그리퍼 | Inspire EG2-4C2 (2지 병렬) |
| 카메라 | Intel RealSense D435 (04부터, 팔 말단 장착) |
| 시뮬레이터 | **새 Gazebo (gz sim)** + MoveIt2 demo (RViz) |

> **참고 (작성 배경)** — 이 가이드는 Ubuntu 24.04 + ROS2 Jazzy 환경에서 실기체로 검증한 과정을
> Humble 환경에 맞게 옮긴 것입니다. 환경 의존 부분은 다음과 같습니다:
> 저장소 브랜치(`jazzy` → `humble`), apt 패키지 접두어(`ros-jazzy-*` → `ros-humble-*`),
> python 버전(3.12 → 3.10, 04의 venv에 영향), pip 옵션(`--break-system-packages` 불필요).
> 선배가 남긴 Jazzy 자료를 참고할 때는 이 부분만 주의하면 됩니다.
>
> ⚠️ **Gazebo는 Jazzy·Humble 모두 "새 Gazebo(gz sim)"입니다.**
> 인터넷의 Humble 자료 대부분이 Gazebo Classic(`gazebo-ros-pkgs`)을 안내하지만,
> **RealMan humble 브랜치는 `gz_ros2_control`을 요구**하므로 Classic을 설치하면
> 02_simulation 2절에서 막힙니다. 다만 배포판 시기에 따라 패키지 이름이
> `gz-` 계열이거나 `ign-` 계열일 수 있어, 01_setup 3절에서 `apt-cache search`로
> 확인한 뒤 설치하도록 안내합니다.

## 문서 구성과 진행 순서

| 순서 | 문서 | 내용 |
|---|---|---|
| 1 | [01_setup](01_setup/README.md) | 환경 구축 — 워크스페이스, 저장소 클론, 의존성, 빌드 |
| 2 | [02_simulation](02_simulation/README.md) | 시뮬레이션 — EG2-4C2 그리퍼 모델 결합 → 모델/Gazebo 실행 → MoveIt2 실습 → 토픽·서비스·액션 |
| 3 | [03_practice](03_practice/README.md) | 실기체 — 네트워크 설정 → 토픽·서비스·액션 → 직접 교시 → 기본 모션 명령(MoveJ·L·J_P) → MoveIt2 → 픽앤플레이스 |
| 4 | [04_handeye_calibration](04_handeye_calibration/README.md) | 비전 준비 — 뎁스카메라 테스트 → YOLO 3D 좌표 → Eye-in-Hand 캘리브레이션 → 진단·검증 |
| 5 | [05_vision_grasp](05_vision_grasp/README.md) | 비전 파지 — 좌표 변환 검증(A→B→C) → 카메라 좌표로 파지 |
| 6 | [06_moveit_grasp](06_moveit_grasp/README.md) | MoveIt 통합 — Scene 등록(D) → 비전 결합(E) → 장애물 회피 실증(F) |

### 스크립트

| 위치 | 내용 |
|---|---|
| [04_handeye_calibration/scripts/](04_handeye_calibration/scripts/) | 비전·캘리브레이션 12개 — `~/robot_vision/`에 복사해 사용 (venv 필요) |
| [05_vision_grasp/scripts/](05_vision_grasp/scripts/) | 비전 파지 검증 3개 (A 정적 → B 접근 → C 파지) |
| [06_moveit_grasp/scripts/](06_moveit_grasp/scripts/) | MoveIt 통합 3개 (D 프리미티브 → E 비전 결합 → F 장애물 도구) |

## 전 과정 공통 원칙

1. **시뮬레이션 선검증** — 실기체에서 실행하는 모든 동작은 반드시 시뮬레이션에서 먼저 검증한다.
2. **Plan과 Execute의 분리** — "주황색은 소원, Plan은 계산, Execute만이 움직임." 잔상(계획 경로)을 눈으로 확인하기 전에는 Execute하지 않는다.
3. **이름은 외우지 말 것** — 토픽·서비스·액션 이름과 메시지 필드는 드라이버/브랜치 버전에 따라 조금씩 다르다. `ros2 topic list`, `ros2 interface show`로 항상 직접 확인하고, 이 가이드의 예시가 실제와 다르면 실제 출력에 맞춰 고친다.
4. **새 터미널마다 source** — `source ~/ros2_ws/install/setup.bash` 누락이 "launch 파일 없음" 오류의 90%다. 01_setup에서 `~/.bashrc`에 등록해 둔다.
5. **여러 줄 명령은 한 줄로 붙여넣기** — `\`로 이어진 명령이 붙여넣기 중 끊기면 `ros2 topic pub`이 **모든 필드를 0으로** 발행한다. 그리퍼의 경우 `position: 0` = 완전히 닫힘이므로 위험하다. 발행 후 `publishing #1:` 줄에 내가 넣은 값이 찍혔는지 매번 확인한다.
6. **venv와 ROS는 섞지 않는다** — 04부터 비전용 python 가상환경(`vision_env`)을 쓴다. colcon 빌드와 `ros2 launch`는 반드시 `deactivate` 상태에서 실행한다. 자세한 것은 04 문서 0절.
7. **안전이 데이터보다, 데이터가 완주보다 우선** — 실기체 수칙은 03_practice 필독. 비상정지는 **웹 UI(`http://192.168.1.18`)가 1차 수단**이다(물리 버튼은 작업 공간 안에 있다).

## 함께 볼 자료

- 연구실 플랫폼 교재 「매니퓰레이터: RM75」 2~7장 — 개념(여자유도, 위치제어 vs 컴플라이언스, 모션 플래닝 3단계 등)은 교재로, 손으로 하는 절차는 이 가이드로.
- RealMan 공식 문서: 저장소의 `rm_doc/` 폴더 및 [공식 GitHub](https://github.com/RealManRobot/ros2_rm_robot/tree/humble)
- 캘리브레이션 도구: [RealManRobot/hand_eye_calibration](https://github.com/RealManRobot/hand_eye_calibration) (04에서 사용, 일부 스크립트는 수정본 제공)

## 검증 상태

| 문서 | Jazzy(작성자 PC) | Humble |
|---|---|---|
| 01_setup | 실기체 검증 완료 | **미검증** — 특히 3절 Gazebo 패키지명 |
| 02_simulation | 실기체 검증 완료 | **미검증** |
| 03_practice | 실기체 검증 완료 (연속 실행 제외) | **미검증** |
| 04_handeye_calibration | 실기체 검증 완료 (오차 6.9 mm) | **미검증** — 스크립트는 배포판 무관 확인 |
| 05_vision_grasp | 실기체 검증 완료 (좌·중·우 파지) | **미검증** — 스크립트는 배포판 무관 확인 |
| 06_moveit_grasp | 실기체 검증 완료 (D·E·F) | **미검증** — moveit_msgs 필드 호환 첫 실행 시 확인 |

> Humble 환경에서 문서와 다르게 동작하는 부분을 발견하면 **멘토에게 공유**해 주세요.
> 그 기록이 다음 기수의 가이드가 됩니다.
