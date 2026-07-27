# 실기체(RM75)와의 관계 · 스택 내부 — 멘토·조교용

> 학생 배포용 아님. 학생에게는 "실기체는 금요일, 멘토 동석 하에서만"
> (교재 2.5·6.1)까지만 전달합니다.

## 개념 매핑 (demo ↔ 실기체)

| 이 저장소 (demo) | 실기체 스택 | 비고 |
|---|---|---|
| mock 컨트롤러(ros2_control GenericSystem) | `rm_driver` + 내장 컨트롤러 | 교재 3.1의 세 층에서 가운데 층 교체 |
| `/rm_group_controller/...` 궤적 액션 | 드라이버의 궤적 실행 경로 | MoveIt 쪽 구성은 동일 사상 |
| 교육용 가상 2지 그리퍼 | 연구실 실물 그리퍼(모델은 멘토 안내) | 폭·속도·힘 개념 동일 — 명령 형식은 실물 기준으로 교체 |
| `grasp()` 항상 성공(흉내) | "목표 폭 전 힘 걸림=파지 성공" 실판정 | 교재 5.3 — 금요일 sim-to-real 관찰 1순위 |
| effort = 0 | 모터 전류 기반 추정치 | 교재 3.3·6.5 기준선 잡기의 대상 |
| MoveJ/L/P 없음(개념만) | 제조사 티칭 SW·`rm_driver` 토픽 | 6.4 멘토 시연 경로 |

## 스택 아키텍처

`rm75_edu_moveit_config` 는 제조사 `rm_75_config`(MoveItConfigsBuilder 표준
구성)를 기반으로 **교육용 그리퍼(병렬 2지, prismatic x2)와 손끝 기준점
`gripper_tcp`(Link7 +z 0.11 m)** 를 더한 것입니다. 플래닝 그룹 `rm_group` 의
tip 을 TCP 로 두어, 마커와 pose goal 이 모두 '손끝' 기준이 되게 했습니다
(교재의 "테이블 위 그 지점" 감각 그대로). 예제의 `ArmClient` 는
`/move_action(plan_only)` + `/execute_trajectory` 로 plan/execute 분리를
코드 구조로 강제합니다(교재 5.1).

## 오프라인 검증 완료 (이 저장소를 만든 환경에서 실측)

- `tools/fk_check.py`: URDF 체인 FK — zero(직립, 공구 z=+Z)·home(TCP
  (0.351, 0, 0.020), 공구 z=−Z) 확인, DOWN_Q=(1,0,0,0) 정합, 예제·미션
  좌표 5점 수치 IK 수렴(잔차 ≤0.1 mm, 관절 한계 내)
- URDF(xacro 평탄화)·SRDF 교차검증: 12링크·구동 9관절, SRDF 참조 누락 0
- moveit_config XML 4·YAML 6 파스, launch·전 파이썬 문법 검증
- 그리퍼 기하: TCP_OFFSET(0.11) = palm(0.05) + finger(0.06) — poses.py 와 동기

## 실환경 미검증 — 최초 1회 점검 절차 (멘토, 배포 전 필수 · 30분)

이 저장소는 ROS/MoveIt 이 없는 환경에서 작성되어 **실행 검증이 되어 있지
않습니다.** 표준 패턴(제조사 config 기반 + Builder 관례)만 사용했지만,
아래를 한 번 통과시킨 뒤 배포하세요.

1. `colcon build --packages-select rm_description rm75_edu_moveit_config rm75_edu`
2. `ros2 launch rm75_edu_moveit_config demo.launch.py` — RViz·모델·마커,
   터미널 오류 없음 (README §3 확인 기준)
3. `ros2 topic list` / `ros2 action list` — README §6 표의 항목 존재
   (특히 gripper_controller 액션, /joint_states 9관절)
4. 마커 Plan/Execute 3곳 + 실패 1곳(체크포인트 ②~④ 리허설)
5. `ros2 run rm75_edu move_home_target` — 홈→목표→홈 완주
6. `ros2 run rm75_edu gripper_demo` — RViz 손가락 개폐 확인
7. `ros2 run rm75_edu mission_scene` → 같은 목표 Plan 이 Scene 을 피하는지
8. `ros2 run rm75_edu pick_place_template` — ①구간 완주(템플릿 상태)
9. (금요일 전) 전체 `colcon build` 로 rm_driver·rm_ros_interfaces 빌드 확인

## 조정 노브

- 미션 좌표·블록 크기: `rm75_edu/poses.py` — 바꾸면 `python3 tools/fk_check.py`
  로 도달성 재검 + `mission_scene`/`pick_place_template` 자동 반영
- 그리퍼 치수: `rm75_edu_moveit_config/config/rm75_edu.urdf.xacro` —
  TCP 오프셋을 바꾸면 poses.py 의 `TCP_OFFSET`·fk_check 의 상수도 함께
- 속도: joint_limits.yaml 의 scaling 0.1 (교재 "실습은 저속" — 올리지 말 것)
- 시작 자세: initial_positions.yaml (= SRDF home)

## 금요일 실기체 준비 (멘토)

- 실기체 브링업(rm_driver 실행·로봇 IP·MoveIt 연동)은 **제조사 문서와
  원본 저장소(RealManRobot/ros2_rm_robot)의 real_moveit_demo 구성 기준**으로
  사전 리허설 필수 — 본 저장소는 demo 모드까지만 검증 대상으로 합니다.
- 교재 6장 SOP 지원: 워크스페이스 제한·보호 등급은 티칭 SW에서 멘토 확인,
  비상정지 전담 지정, 속도 스케일 저속 유지, 직접 교시 모드 전환은 멘토만.
- 실물 그리퍼 연동 형식이 확정되면 `GripperClient` 의 demo 경고 문구와
  grasp 판정을 실물 기준으로 교체.
