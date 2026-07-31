# RM75 로봇팔 실습 가이드 (Ubuntu 24.04 + ROS2 Jazzy) — 교육 전 사전 테스트판

후배 배포용 `ros2_humble_guide`와 내용·순서가 동일하고, 환경 의존 부분만 작성자 PC(Jazzy)에 맞춘 버전입니다.
[RealMan 공식 저장소](https://github.com/RealManRobot/ros2_rm_robot)의 **jazzy 브랜치**를 기준으로 하며,
**시뮬레이션 검증 → 실기체 실행** 순서로 진행합니다.

## 대상 환경

| 항목 | 값 |
|---|---|
| OS | Ubuntu 24.04 |
| ROS2 | Jazzy |
| 로봇 | RealMan RM75 (7자유도 협동 로봇 팔) |
| 그리퍼 | Inspire EG2-4C2 (2지 병렬) |
| 시뮬레이터 | 새 Gazebo (gz sim, Harmonic) + MoveIt2 demo (RViz) |

> **참고 (Humble 배포본과의 차이)** — 환경 의존 부분은 딱 네 가지입니다:
> 저장소 브랜치(`humble` → `jazzy`), Gazebo(**Gazebo Classic** `gazebo-ros` 계열 → 새 Gazebo `ros-gz` 계열),
> apt 패키지 접두어(`ros-humble-*` → `ros-jazzy-*`), pip(24.04는 `--break-system-packages` 필요 — 이 가이드는 pip 미사용).
> 리허설 중 문제가 나면, 이 네 가지에 해당하는 부분인지(= 후배 환경에서는 다르게 겪을 부분인지)부터 확인하면 됩니다.

## 문서 구성과 진행 순서

| 순서 | 문서 | 내용 |
|---|---|---|
| 1 | [01_setup](01_setup/README.md) | 환경 구축 — 워크스페이스, 저장소 클론, 의존성, 빌드 |
| 2 | [02_simulation](02_simulation/README.md) | 시뮬레이션 — EG2-4C2 그리퍼 모델 결합 → 모델/Gazebo 실행 → MoveIt2 실습 → 토픽·서비스·액션 |
| 3 | [03_practice](03_practice/README.md) | 실기체 — 네트워크 설정 → 토픽·서비스·액션 → 직접 교시 → 기본 모션 명령(MoveJ·L·J_P) → MoveIt2 → 픽앤플레이스 |
| - | [scripts/](scripts/) | 실습용 파이썬 스크립트 (pick_place, grasp_test, real_pick_place) |

## 전 과정 공통 원칙

1. **시뮬레이션 선검증** — 실기체에서 실행하는 모든 동작은 반드시 시뮬레이션에서 먼저 검증한다.
2. **Plan과 Execute의 분리** — "주황색은 소원, Plan은 계산, Execute만이 움직임." 잔상(계획 경로)을 눈으로 확인하기 전에는 Execute하지 않는다.
3. **이름은 외우지 말 것** — 토픽·서비스·액션 이름과 메시지 필드는 드라이버/브랜치 버전에 따라 조금씩 다르다. `ros2 topic list`, `ros2 interface show`로 항상 직접 확인하고, 이 가이드의 예시가 실제와 다르면 실제 출력에 맞춰 고친다.
4. **새 터미널마다 source** — `source ~/ros2_ws/install/setup.bash` 누락이 "launch 파일 없음" 오류의 90%다. 01_setup에서 `~/.bashrc`에 등록해 둔다.
5. **안전이 데이터보다, 데이터가 완주보다 우선** — 실기체 수칙은 03_practice 0절 필독.

## 함께 볼 자료

- 연구실 플랫폼 교재 「매니퓰레이터: RM75」 2~7장 — 개념(여자유도, 위치제어 vs 컴플라이언스, 모션 플래닝 3단계 등)은 교재로, 손으로 하는 절차는 이 가이드로.
- RealMan 공식 문서: 저장소의 `rm_doc/` 폴더 및 [공식 GitHub](https://github.com/RealManRobot/ros2_rm_robot/tree/jazzy)
