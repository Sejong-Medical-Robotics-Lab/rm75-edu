"""rm75_edu.poses — 저장소 공용 상수 (tools/fk_check.py 로 도달성 검증된 값).

여기 숫자를 바꾸면 fk_check.py 를 다시 돌려 도달성을 재확인할 것 —
이 파일과 fk_check.py 는 같은 값을 가져야 한다(단일 기준).
"""

# 플래닝 그룹 이름 (SRDF)
ARM_GROUP = "rm_group"
GRIPPER_GROUP = "gripper"

# 자주 쓰는 팔 자세 (joint1..joint7 [rad]) — SRDF group_state 와 동일
HOME = [0.0, 0.8395, 0.0, 1.4442, 0.0, 0.8513, 0.0]   # 테이블 내려보기 준비 자세
ZERO = [0.0] * 7                                        # 직립

ARM_JOINTS = [f"joint{i}" for i in range(1, 8)]
GRIPPER_JOINTS = ["gripper_finger1_joint", "gripper_finger2_joint"]

# '아래 보기' 자세 상수 (x, y, z, w) — 공구 +z 가 월드 -z (교재 5.2:
# 자세는 새로 만들지 말고 검증된 상수를 재사용)
DOWN_Q = (1.0, 0.0, 0.0, 0.0)

# pose goal 은 gripper_tcp(손끝) 기준, base_link 좌표계 [m]
TCP_OFFSET = 0.11           # Link7 → TCP (참고용 — MoveIt 그룹이 이미 TCP 기준)

# 그리퍼 (교재 5.3의 세 숫자)
GRIPPER_WIDTH_MAX = 0.08    # 두 손가락 사이 최대 폭 [m] (관절값 x 2)
GRIPPER_WIDTH_MIN = 0.0

# ── 미션 좌표 (7장) — demo Scene(03_mission_scene)과 동기 ──────────────
TABLE_TOP_Z = -0.005        # 테이블 상판 윗면 z (베이스 원점보다 5mm 아래)
BLOCK_SIZE = 0.04           # 부드러운 블록 한 변 [m]
A_XY = (0.34, 0.16)         # 집는 위치 A (x, y)
B_XY = (0.34, -0.16)        # 놓는 위치 B
APPROACH_Z = 0.15           # 접근 자세 TCP 높이 (물체 위 안전 높이 — 7.3 힌트)
GRASP_Z = 0.02              # 파지 시 TCP 높이 (블록 중심 부근)
PLACE_Z = 0.03              # 내려놓기 TCP 높이
