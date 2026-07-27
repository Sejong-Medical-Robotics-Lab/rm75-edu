# RM75 관절 지도 (교재 2.1 ↔ /joint_states)

`/joint_states` 의 배열은 **name 순서가 정답지**입니다(교재 3.3) — 인덱스를
외우지 말고 name 으로 짝지어 읽으세요. 아래는 name ↔ 사람 팔 대응입니다.

| name | 교재 2.1 | 사람 팔 대응 |
|---|---|---|
| `joint1` | J1 (어깨) | 팔 전체 좌우 회전 |
| `joint2` | J2 (어깨) | 팔 앞뒤 들어올림 |
| `joint3` | J3 (어깨) | 위팔 비틀기 |
| `joint4` | J4 (팔꿈치) | 접었다 펴기 — 여자유도의 주인공 |
| `joint5` | J5 (손목) | 아래팔 비틀기 |
| `joint6` | J6 (손목) | 손목 꺾기 |
| `joint7` | J7 (손목) | 말단 회전 |
| `gripper_finger1_joint` | 그리퍼 | 손가락 1 (0~0.04 m, 폭의 절반) |
| `gripper_finger2_joint` | 그리퍼 | 손가락 2 (대칭) |

- 관절 범위·속도 한계는 교재 2.6 사양표와 `rm_description/urdf` 기준.
- 플래닝 그룹: `rm_group`(joint1~7, tip=`gripper_tcp`) · `gripper`(손가락 2).
