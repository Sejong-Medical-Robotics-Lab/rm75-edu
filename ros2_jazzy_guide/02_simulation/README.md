# 02. 시뮬레이션 — 그리퍼 결합부터 MoveIt2, 토픽·서비스·액션까지

**목표**
1. 실습 기체와 동일하게 **EG2-4C2 그리퍼가 결합된 RM75 모델**을 만들고 RViz로 확인한다.
2. Gazebo(물리)와 MoveIt2 demo(운동학)를 띄우고, **Plan → 잔상 확인 → Execute** 흐름을 몸에 붙인다.
3. 그리퍼 포함 MoveIt 설정(`rm_75_jaw_config`)을 Setup Assistant로 직접 만든다.
4. 시뮬레이션 팔을 상대로 **토픽·서비스·액션**을 직접 다뤄 본다.

> 💡 **파일 이름이 다를 때** — launch·URDF 파일명은 브랜치/버전에 따라 조금 다를 수 있다.
> 명령이 실패하면 당황하지 말고 `ls ~/ros2_ws/src/ros2_rm_robot/<패키지>/launch/`(또는 `/urdf/`)로
> 실제 파일명을 먼저 확인한다.

> 💡 **`cat > ... << 'EOF'` 블록을 붙여넣을 때** — 이 문서에는 파일을 통째로 만드는 명령이 여러 번 나온다.
> 터미널에 붙여넣으면 화면이 겹쳐 보이거나 중간에서 끊길 수 있다. 붙여넣은 뒤 반드시
> `tail -5 <파일>`로 마지막 줄까지 들어갔는지 확인하고, 깨졌다면 `nano <파일>`로 직접 편집한다.

---

## 1. EG2-4C2 그리퍼 모델 결합

**왜 하는가** — 기본 `rm_description`에는 팔만 있다. 우리 실습 기체에는 Inspire
[EG2-4C2](https://en.inspire-robots.com/product/eg2-4c) 2지 그리퍼가 장착되어 있으므로,
시뮬레이션도 같은 구성으로 맞추고, 파지 기준점(TCP) 프레임까지 미리 심어 둔다.
(그리퍼 유무에 따라 충돌 판정·파지점 계산이 완전히 달라진다.)

### 1-1. 그리퍼 모델 파일 가져오기

RealMan이 공개한 URDF 변환 저장소에 EG2-4C2가 결합된 모델의 압축 파일이 들어 있다.
여기서 메시(STL)와 그리퍼 xacro만 뽑아 `rm_description`에 넣는다.

```bash
cd /tmp
git clone https://github.com/RealManRobot/URDF-to-XACRO.git
cd URDF-to-XACRO
unzip -o rm_Lifting_robot_75B_jaw_description.zip

SRC=rm_Lifting_robot_75B_jaw_description
DST=~/ros2_ws/src/ros2_rm_robot/rm_description

cp $SRC/meshes/4C2_*.STL $DST/meshes/
cp $SRC/urdf/jaw.urdf.xacro $DST/urdf/

# 메시 경로를 rm_description 기준으로 변경
sed -i 's|package://rm_Lifting_robot_75B_jaw_description/meshes/|package://rm_description/meshes/|' \
   $DST/urdf/jaw.urdf.xacro

# 확인: 경로가 바뀌었는지, STL이 7개인지
grep mesh_path $DST/urdf/jaw.urdf.xacro   # package://rm_description/meshes/ 로 바뀌어야 함
ls $DST/meshes/4C2_*.STL | wc -l          # 7 이 나와야 함
```

> 💡 압축 파일 안에는 이동 대차(AGV)·리프팅 몸체까지 들어 있지만, 우리가 쓰는 것은
> **그리퍼 부분(`4C2_*`)뿐**이다. 그래서 STL 7개와 `jaw.urdf.xacro` 하나만 뽑아 온다.

### 1-2. 팔 + 그리퍼 결합 URDF 만들기

팔 본체와 그리퍼를 include하고, 고정 조인트로 잇는다. **TCP 프레임(`grasp_tcp`)도 여기서 함께
추가**한다 — 나중에 픽앤플레이스에서 "손가락 패드 중앙"을 목표점으로 쓰기 위한 프레임이다.

```bash
cat > ~/ros2_ws/src/ros2_rm_robot/rm_description/urdf/rm_75_with_jaw.urdf.xacro << 'EOF'
<?xml version="1.0"?>
<robot name="rm_75_description" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- RM75 팔 본체 -->
  <xacro:include filename="$(find rm_description)/urdf/rm_75.urdf.xacro" />

  <!-- Inspire EG2-4C2 그리퍼 -->
  <xacro:include filename="$(find rm_description)/urdf/jaw.urdf.xacro" />

  <!-- 말단(Link7)에 그리퍼 장착 -->
  <joint name="arm_jaw_joint" type="fixed">
    <origin xyz="0 0 -0.009" rpy="0 0 -1.57" />
    <parent link="Link7" />
    <child link="4C2_baselink" />
  </joint>

  <!-- 파지 기준점(TCP): 두 손가락 패드 중앙 -->
  <link name="grasp_tcp"/>
  <joint name="grasp_tcp_joint" type="fixed">
    <origin xyz="0 0 0.12" rpy="0 0 0" />
    <parent link="Link7" />
    <child link="grasp_tcp" />
  </joint>

</robot>
EOF
```

- `xyz="0 0 -0.009"` : 그리퍼 베이스와 플랜지 면을 맞추는 실측 보정값. 값을 잘못 주면(예: `0.03`)
  RViz에서 그리퍼가 팔에서 **떨어져 떠 있는 것처럼** 보인다 — 그때는 이 z값을 조금씩 조정한다.
- `rpy="0 0 -1.57"` : 그리퍼 개폐 방향 정렬.
- `grasp_tcp` : Link7(플랜지)에서 z축 +0.12 m — 두 손가락 패드 중앙. **근거를 직접 확인하려면**
  1-4에서 RViz를 띄운 뒤 **그리퍼를 닫은 상태**(`jaw_Joint1 = 0`)에서:

  ```bash
  ros2 run tf2_ros tf2_echo Link7 4C2_Link5    # 패드 링크: z≈0.106, y≈0.018
  ```

  패드가 z=0.106에서 시작하므로 0.12는 패드 구간 안에 든다.
  (첫 줄의 `Invalid frame ID "Link7"`은 TF 트리가 채워지기 전 한 번 뜨는 것이므로 무시)
  실물 실측 후 미세 조정 가능하며, **이 값이 픽앤플레이스 파지 정확도를 좌우한다.**
- 팔 본체 include의 파일명(`rm_75.urdf.xacro`)이 다르면
  `ls ~/ros2_ws/src/ros2_rm_robot/rm_description/urdf/`로 실제 이름을 확인해 맞춘다.

> 💡 **그리퍼 조인트는 6개지만 실제로 구동되는 것은 `jaw_Joint1` 하나다.**
> 나머지 5개(`jaw_Joint2`~`jaw_Joint6`)는 `<mimic>`으로 `jaw_Joint1`을 따라가는 링키지다
> — 모터 하나로 두 손가락이 함께 움직이는 구조다. 그래서 이후 슬라이더에도, MoveIt 설정에도
> `jaw_Joint1`만 나타난다.

### 1-3. 빌드 & 검증 (RViz 켜기 전에)

```bash
cd ~/ros2_ws
colcon build --packages-select rm_description
source install/setup.bash

# xacro → URDF 전개 후 트리 구조 검증
xacro $(ros2 pkg prefix rm_description)/share/rm_description/urdf/rm_75_with_jaw.urdf.xacro > /tmp/rm75_jaw.urdf
check_urdf /tmp/rm75_jaw.urdf

# 설치된 메시 확인
ls $(ros2 pkg prefix rm_description)/share/rm_description/meshes/4C2_*.STL | wc -l   # 7
```

`check_urdf`가 `base_link`를 루트로 하는 링크 트리를 출력하고 오류가 없으면 통과.
**여기서 실패하면 RViz를 켜도 안 된다** — 오류 메시지에 나온 링크/조인트 이름부터 고친다.

트리 끝부분이 아래처럼 나오면 결합이 제대로 된 것이다 (`Link7`에 자식이 둘):

```
                            child(1):  Link7
                                child(1):  4C2_baselink
                                    ...
                                child(2):  grasp_tcp
```

### 1-4. 표시용 launch 만들기 + 눈으로 확인

```bash
cat > ~/ros2_ws/src/ros2_rm_robot/rm_description/launch/rm_75_jaw_display.launch.py << 'PYEOF'
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('rm_description')
    xacro_file = os.path.join(share, 'urdf', 'rm_75_with_jaw.urdf.xacro')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str,
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'rm_75.rviz')],
        ),
    ])
PYEOF

python3 -m py_compile ~/ros2_ws/src/ros2_rm_robot/rm_description/launch/rm_75_jaw_display.launch.py && echo "문법 OK"

cd ~/ros2_ws
colcon build --packages-select rm_description
source install/setup.bash
ros2 launch rm_description rm_75_jaw_display.launch.py
```

**확인 포인트**

- [ ] RViz에 팔 + 그리퍼가 붙어서 보인다 (떠 있거나 파묻혀 있으면 1-2의 origin z 조정)
- [ ] 슬라이더 창에 관절 8개: `joint1`~`joint7` + `jaw_Joint1`
      (mimic 조인트 5개는 GUI가 자동으로 숨긴다)
- [ ] `jaw_Joint1` 슬라이더로 그리퍼가 열리고 닫힌다 (0 = 닫힘, 0.82 = 최대 열림)
- [ ] RViz의 Displays → TF를 켜면 `grasp_tcp` 프레임이 손가락 사이에 보인다
- [ ] 터미널의 `[kdl_parser] The root link base_link has an inertia ...` **WARN은 정상**이다
      — RealMan 원본 URDF에서 오는 것으로 표시·계획에 영향이 없다. 이후 문서에서도 계속 나타난다.
- 로봇이 안 보이면: Fixed Frame이 `base_link`인지, RobotModel 디스플레이가 켜져 있는지 확인

<details>
<summary><b>(참고) 팔 단독 모델만 보고 싶을 때</b></summary>

그리퍼 결합에 문제가 생겼을 때, 팔 자체는 정상인지 분리해 확인하는 용도.

```bash
# 터미널 1
ros2 launch rm_description rm_75_display.launch.py
# 터미널 2 (슬라이더가 자동으로 안 뜨는 경우)
ros2 run joint_state_publisher_gui joint_state_publisher_gui
# 터미널 3
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix rm_description)/share/rm_description/rviz/rm_75.rviz
```
</details>

---

## 2. Gazebo 시뮬레이션 (물리)

```bash
ros2 launch rm_gazebo gazebo_75_demo.launch.py
```

- Jazzy에서는 **새 Gazebo(gz sim)** 창이 뜬다. **첫 실행은 모델·리소스 로딩 때문에 1~2분 걸릴 수 있다** — 기다린다.
- **그리퍼가 보이지 않는 것이 정상이다.** Gazebo는 `rm_gazebo/config/gazebo_75_description.urdf.xacro`라는
  **별도의 모델 파일**을 쓰며, 1절에서 `rm_description`에 만든 결합 모델과는 다른 파일이다.
  그리퍼까지 포함한 계획은 3-4에서 만들 `rm_75_jaw_config`에서 다룬다.

새 터미널에서 컨트롤러와 상태 발행 주기를 확인:

```bash
ros2 control list_controllers   # joint_state_broadcaster, rm_group_controller 모두 active
ros2 topic hz /joint_states     # 100 Hz (ros2_controllers.yaml의 update_rate와 일치)
```

> 💡 이 **100 Hz**는 03_practice에서 실기체 값과 비교하게 된다. 적어 두자.

> 🔧 Gazebo가 비정상 종료되어 다시 안 뜰 때 (새 Gazebo는 `gzserver`/`gzclient` 프로세스가 아니다):
> ```bash
> pkill -9 -f "gz sim"
> ```

### 💡 RViz와 Gazebo는 역할이 다르다

|  | RViz | Gazebo |
|---|---|---|
| 역할 | **시각화** (받아서 그림) | **물리 시뮬레이션** (계산해서 내보냄) |
| 중력·관성·마찰 | 없음 | 있음 |
| `/joint_states` | 구독만 | 발행 |

RViz에서 팔이 움직이는 것은 RViz가 계산해서가 아니라, **누군가 발행한 `/joint_states`를 그대로 그리는 것**이다.
그 값을 만드는 주체는 상황마다 다르다.

```
MoveIt ──액션──▶ joint_trajectory_controller ──▶ [하드웨어 인터페이스] ──▶ 팔
                                                        ↑
                              여기만 갈아끼운다: FakeSystem / GazeboSim / 실기체
```

- **3-2·3-5의 demo** → `mock_components/GenericSystem`(FakeSystem). 명령을 그대로 상태로 되돌려주므로 오차가 없다.
- **2절·3-3의 Gazebo** → `gz_ros2_control/GazeboSimSystem`. 중력·관성을 계산하므로 추종 오차가 생긴다
  (3-3에서 허용 오차를 완화하는 이유가 이것이다).
- **03_practice의 실기체** → RealMan 하드웨어 인터페이스.

**위 두 층(MoveIt, 컨트롤러)은 세 경우 모두 동일하다.** 토픽·액션·컨트롤러 이름이 바뀌지 않으므로,
4절에서 익힌 명령이 03의 실기체에서 그대로 통한다.

---

## 3. MoveIt2 실습

### 3-1. 개념 요약 (교재 4장)

- 모션 플래닝 = **① 역기구학(IK) → ② 경로 계획(장애물 회피) → ③ 시간 궤적화**.
- 화면의 문법: **주황색 로봇 = 목표(아직 소원), Plan = 계산, Execute만이 실제 움직임.**
- MoveIt2의 기본 플래너(OMPL)는 확률적 탐색이라 **같은 목표라도 매번 다른 경로**가 나올 수 있다
  → 그래서 실기체에서는 Execute 전 잔상 확인을 절대 생략할 수 없다.
- Planning Scene에 등록되지 않은 실물은 플래너에게 **보이지 않는다.**

> ⚠️ **RViz 종료 주의 (3장 전체 공통)** — Planning Scene에 충돌 물체(박스 등)가 있는 상태에서
> **주황색 마커를 드래그하면 RViz가 종료된다**(세그폴트). demo 모드와 Gazebo 연동 양쪽에서
> 재현되며, 그래픽 드라이버와는 무관하다.
>
> **아래 순서를 지키면 발생하지 않는다:**
> 1. 마커로 목표를 먼저 정하고 **Plan**
> 2. 그 다음에 박스를 추가
> 3. 마커는 그대로 두고 **Plan만 다시**
>
> 목표를 바꾸려면 **박스를 먼저 지운 뒤** 마커를 움직인다.
> 종료되어도 저장된 것은 손상되지 않으므로 launch를 다시 실행하면 된다.

### 3-2. 기본 demo (팔 단독) — 마우스로 하는 플래닝

```bash
ros2 launch rm_75_config demo.launch.py
```

물리 엔진 없이 가상 컨트롤러(mock)가 궤적을 재생하는 **운동학 시뮬레이션**이다.
아래 체크리스트를 순서대로 완주한다:

| # | 할 일 | 통과 기준 |
|---|---|---|
| ① | 주황 마커(공=위치, 고리=자세)를 끌어 목표 지정 → **Plan** | 서로 다른 3곳에서 계획 성공 (잔상 애니메이션) |
| ② | Plan 성공한 계획을 **Execute** | 이동 후 로봇 자세 = 방금의 주황색 자세 |
| ③ | 같은 목표를 5회 **Plan만** 반복 (**Execute 금지** — 실행하면 출발 자세가 바뀌어 조건이 달라진다) | 잔상 경로가 매번 다른 것 관찰 (플래너의 무작위성) |
| ④ | 마커로 목표 지정 → **Plan** → Scene 탭에서 박스를 경로에 걸치게 추가 → **마커는 그대로 두고 Plan만 다시** | 같은 목표인데 경로가 장애물을 피해 달라짐 |
| ⑤ | ④의 박스를 **주황색 목표 로봇과 겹치도록** 옮기고 Plan | 계획 **실패**. 좌측에 `Failed`, 터미널에 실패 메시지 |
| ⑥ | Planning 탭의 Goal State에서 named target(`zero` / `forward`) 선택 → Plan & Execute | 미리 정의된 자세로 이동 |

> ⚠️ ④⑤에서 **박스를 추가한 뒤에는 마커를 드래그하지 말 것** (3-1의 공통 주의사항 참고).
> 목표를 바꾸려면 박스를 먼저 지운다.

⑤의 터미널 메시지를 꼭 읽어 보자:

```
Unable to find solution by any of the threads in 0.0002 seconds
Planner 'OMPL' failed with error code FAILURE
```

**Planning Time 5초를 다 쓰지도 않고 즉시 실패**한 점이 핵심이다. 목표 자세 자체가 장애물과
충돌하므로 탐색을 시작할 수조차 없다 — "도달은 가능하지만 계획은 불가능"한 상태다.

> 💡 RViz의 마커는 **IK가 풀리는 범위 밖으로는 끌리지 않는다.** 그래서 "너무 먼 곳"으로 옮겨
> 실패를 만드는 것은 불가능하고, 위처럼 장애물을 이용해야 한다.

이 SRDF에 정의된 named target은 `zero`와 `forward` **두 개**다. 직접 확인:

```bash
grep -A10 'group_state' ~/ros2_ws/src/ros2_rm_robot/rm_moveit2_config/rm_75_config/config/rm_75_description.srdf
```

### 3-3. Gazebo + MoveIt2 연동 — 계획이 물리 세계에서 실행되는가

**사전 조정 (권장)** — Gazebo 물리 시뮬에서는 궤적 추종 오차가 생기는데, 컨트롤러의 허용 오차가
너무 빡빡하면 실행이 중간에 중단된다(`PATH_TOLERANCE_VIOLATED`). 미리 완화해 둔다:

```bash
sed -i 's/trajectory: 0.05/trajectory: 0.1/' \
  ~/ros2_ws/src/ros2_rm_robot/rm_moveit2_config/rm_75_config/config/ros2_controllers.yaml

# 반영 확인 (trajectory: 0.1 이 보이면 성공. 해당 줄이 원래 없다면 이 단계는 건너뜀)
grep -A2 "joint1:" ~/ros2_ws/src/ros2_rm_robot/rm_moveit2_config/rm_75_config/config/ros2_controllers.yaml

# yaml은 스폰 시점에 읽히므로 빌드 필수
cd ~/ros2_ws
colcon build --packages-select rm_75_config
source install/setup.bash
```

> 💡 Gazebo가 이 파일을 읽는 이유 — `rm_gazebo`의 모델 xacro가
> `$(find rm_75_config)/config/ros2_controllers.yaml`을 참조하도록 되어 있다.
> **install 경로에서 읽으므로 수정 후 재빌드가 반드시 필요하다.**

실행 (내부적으로 Gazebo를 먼저 띄운 뒤 MoveIt을 연결한다):

```bash
ros2 launch rm_bringup rm_75_gazebo.launch.py
```

- RViz에서 Plan & Execute → **Gazebo 안의 팔이 실제로 움직이는지** 확인.
  두 화면의 팔은 서로 다른 로봇이 아니라, **같은 `/joint_states`를 Gazebo가 발행하고 RViz가 그리는 것**이다.
- RViz Planning 탭의 **Velocity/Accel Scaling이 0.10인지 확인**한다 — 설정 파일의
  `default_velocity_scaling_factor` 값이 반영된 기본값이다. 실기체에서도 이 값으로 시작하며,
  올리더라도 0.3을 넘기지 않는 습관을 여기서부터 들인다.
- 새 터미널에서 `ros2 control list_controllers`로 컨트롤러 active 재확인 가능.

### 3-4. 그리퍼 포함 MoveIt 설정 만들기 — Setup Assistant로 `rm_75_jaw_config` 생성

1절에서 만든 결합 모델(`rm_75_with_jaw.urdf.xacro`)로 **그리퍼까지 계획 대상에 포함하는**
MoveIt 설정 패키지를 만든다. 뒤의 픽앤플레이스 시뮬레이션이 전부 이 위에서 돈다.

앞 절의 Gazebo·RViz는 모두 닫고 시작한다.

```bash
pkill -9 -f "gz sim"
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

**Setup Assistant 진행 순서** (화면 좌측 탭 순서대로):

1. **Create New MoveIt Configuration Package**
   - URDF 경로: `/home/<사용자명>/ros2_ws/install/rm_description/share/rm_description/urdf/rm_75_with_jaw.urdf.xacro`
     (반드시 **install 경로** — 그래서 1-3에서 빌드를 먼저 했다. `~`는 GUI에서 풀리지 않으니 절대 경로로 입력)
     → Load Files → 우측 3D 뷰에 **그리퍼가 붙은 팔**이 보이면 성공
2. **Self-Collisions** — Sampling Density 기본값 → Generate Collision Matrix

   > 💡 `4C2_Link5` ↔ `4C2_Link6`이 **분홍색(Default in collision)** 으로 잡히는 것은 정상이다.
   > 그리퍼가 완전히 닫힌 기본 자세에서 두 손가락 패드가 맞닿기 때문이다. 체크된 상태
   > (= 충돌 검사 제외)로 두어야 이후 `closed` 자세의 계획이 성공한다.
   > 하단 로그에 `Always in collision : 0`이 나오면 모델에 결함이 없다는 뜻이다.
3. **Virtual Joints** (**권장**) — Name `virtual_joint` / Child `base_link` / Parent `world` / Type `fixed`

   > 💡 없어도 동작하지만, 실행할 때마다 `No root/virtual joint specified in SRDF` 경고가 뜬다.
   > 넣어 두면 사라지고, 픽앤플레이스에서 `world` 기준 좌표를 쓸 때도 깔끔하다.
4. **Planning Groups** — 두 개 생성
   - `rm_group` : Kinematic Solver `kdl_kinematics_plugin/KDLKinematicsPlugin`,
     Search Resolution 0.005, Timeout 0.05, Group Default Planner `RRTConnect`
     → **Add Kin. Chain**: Base Link `base_link`, Tip Link `Link7` → Save
   - `gripper` : Kinematic Solver **None**
     → **Add Joints**: `jaw_Joint1`만 선택 → Save
     (목록에 `jaw_Joint2`~`6`은 나타나지 않는다 — mimic 조인트라 자동으로 걸러진다)
5. **Robot Poses** — 4개 등록
   - `zero` (rm_group, 슬라이더 전부 0)
   - `forward` (rm_group) — 기존 `rm_75_config`와 같은 값을 입력한다:
     `joint2 = 0.8395`, `joint4 = 1.4442`, `joint6 = 0.8513`, 나머지 0
     (같은 이름이 같은 자세를 가리켜야 두 패키지를 오갈 때 헷갈리지 않는다)
   - `open` (gripper, `jaw_Joint1` = 0.82 — 슬라이더 최대)
   - `closed` (gripper, `jaw_Joint1` = 0)
6. **End Effectors** — Name `gripper_ee` / Group `gripper` / Parent Link `Link7` / Parent Group `rm_group`
7. **ros2_control URDF Modification** — Command Interfaces ☑ position,
   State Interfaces ☑ position ☑ velocity → **Add interfaces**
   관절 목록에 `joint1`~`joint7` + `jaw_Joint1` **총 8개**가 모두 포함되어야 한다.
8. **ROS 2 Controllers** — "Auto Add JointTrajectoryController Controllers For Each Planning Group" 클릭
   → `rm_group_controller`(7관절), `gripper_controller`(`jaw_Joint1`) 생성 확인
9. **MoveIt Controllers** — "Auto Add FollowJointTrajectory Controllers For Each Planning Group" 클릭
10. **Author Information** — 이름·이메일 **필수** (비우면 마지막에 생성 거부됨. 아무 값이나 가능)
11. **Configuration Files** — 저장 경로를 입력란에 **직접 타이핑**한다.
    아직 존재하지 않는 폴더이므로 **Browse로는 찾을 수 없다.**
    공백 없는 절대 경로: `/home/<사용자명>/ros2_ws/src/rm_75_jaw_config`

    > ⚠️ **`ros2_rm_robot/` 안에 만들지 말 것** — 우리가 만든 패키지를 RealMan 저장소 안에 두면
    > `git pull` 시 충돌하고, 저장소를 다시 클론하면 사라진다. 반드시 `src/` **바로 아래**에 만든다.
12. **Generate Package** 클릭 → 진행률 **100% 확인** → 그 **다음에** Exit Setup Assistant

    > ⚠️ 두 버튼이 나란히 있다. **Generate Package를 먼저 누르지 않고 Exit하면 아무것도 생성되지 않으며,
    > 설정도 저장되지 않아 1단계부터 다시 해야 한다.**

생성 위치 확인:

```bash
find ~/ros2_ws/src -maxdepth 3 -name "rm_75_jaw_config" -type d
ls ~/ros2_ws/src/rm_75_jaw_config    # CMakeLists.txt  config  launch  package.xml
```

경로가 `~/ros2_ws/src/rm_75_jaw_config`가 아니면 `mv`로 옮긴다.

### 3-5. 생성 직후 필수 보정 2가지 (첫 실행 전에!)

Setup Assistant가 생성한 파일 그대로 실행하면 두 가지 문제가 난다.
**첫 실행 전에** 아래 두 파일을 덮어써서 예방한다.

**① `joint_limits.yaml`** — Setup Assistant가 `max_acceleration: 0`(전 관절),
`jaw_Joint1`의 `max_velocity: 1`(정수)로 생성한다. 이대로 두면 **경로 계획은 성공하지만
시간 궤적화 단계에서** `No acceleration limit was defined for joint jaw_Joint1` 오류로 실패한다.
아래로 교체:

```bash
cat > ~/ros2_ws/src/rm_75_jaw_config/config/joint_limits.yaml << 'EOF'
# joint_limits.yaml allows the dynamics properties specified in the URDF to be overwritten or augmented as needed
default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1

joint_limits:
  jaw_Joint1:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 2.0
  joint1:
    has_velocity_limits: true
    max_velocity: 3.14
    has_acceleration_limits: true
    max_acceleration: 5.0
  joint2:
    has_velocity_limits: true
    max_velocity: 3.14
    has_acceleration_limits: true
    max_acceleration: 5.0
  joint3:
    has_velocity_limits: true
    max_velocity: 3.92
    has_acceleration_limits: true
    max_acceleration: 5.0
  joint4:
    has_velocity_limits: true
    max_velocity: 3.92
    has_acceleration_limits: true
    max_acceleration: 5.0
  joint5:
    has_velocity_limits: true
    max_velocity: 3.92
    has_acceleration_limits: true
    max_acceleration: 5.0
  joint6:
    has_velocity_limits: true
    max_velocity: 3.92
    has_acceleration_limits: true
    max_acceleration: 5.0
  joint7:
    has_velocity_limits: true
    max_velocity: 3.92
    has_acceleration_limits: true
    max_acceleration: 5.0
EOF

tail -5 ~/ros2_ws/src/rm_75_jaw_config/config/joint_limits.yaml   # joint7까지 들어갔는지 확인
```

> ⚠️ `has_acceleration_limits`는 반드시 **`true`** 여야 한다. `false`로 두면 아래
> `max_acceleration` 값이 무시되어 위 오류가 그대로 발생한다.

> 💡 이 실패는 3-1에서 배운 "① IK → ② 경로 계획 → ③ 시간 궤적화" 중 **③에서만 걸린 것**이다.
> 터미널 로그에서 `Calling Planner 'OMPL'` 다음에는 에러가 없다가
> `AddTimeOptimalParameterization`에서 실패하는 순서를 확인해 보자.

**② `moveit_controllers.yaml`** — Setup Assistant 생성본에는 `action_ns`와 `default`가 빠져 있다.
`action_ns`가 없으면 MoveIt이 컨트롤러의 액션 서버 이름을 조립하지 못해 **Plan은 되는데 Execute가
실패한다.** 교체:

```bash
cat > ~/ros2_ws/src/rm_75_jaw_config/config/moveit_controllers.yaml << 'EOF'
# MoveIt uses this configuration for controller management
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - rm_group_controller
    - gripper_controller

  rm_group_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
      - joint7

  gripper_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - jaw_Joint1
EOF

grep -c action_ns ~/ros2_ws/src/rm_75_jaw_config/config/moveit_controllers.yaml   # 2 가 나와야 함
```

빌드 후 실행:

```bash
cd ~/ros2_ws
colcon build --packages-select rm_75_jaw_config
source install/setup.bash
ros2 launch rm_75_jaw_config demo.launch.py
```

**확인 포인트**

- [ ] 팔 + 그리퍼 모델이 표시된다
- [ ] MotionPlanning → Planning Group을 `rm_group` ↔ `gripper`로 전환 가능
- [ ] `gripper` 그룹에서 Goal State `open` / `closed` 선택 → Plan & Execute → 그리퍼 개폐
- [ ] `rm_group`에서 마커 이동 → Plan & Execute 정상
- [ ] 새 터미널에서 `ros2 control list_controllers` → `joint_state_broadcaster`,
      `rm_group_controller`, `gripper_controller` **3개 모두 active**
- 💡 그리퍼가 **비대칭/유령처럼 겹쳐 보이면**: Displays → MotionPlanning → Planning Request →
  **Query Goal State 체크 해제** (목표 상태의 잔상이 겹쳐 보이는 것으로, 버그가 아님)

### 3-6. MoveIt 실습 트러블슈팅

| 증상 | 원인 / 처방 |
|---|---|
| `No acceleration limit was defined for joint ...` | 3-5 ①의 joint_limits.yaml 보정 누락, 또는 `has_acceleration_limits`가 `false` → 수정 후 **재빌드** |
| Plan 성공, Execute 실패 | 3-5 ②의 `action_ns` 누락 → 보정 후 재빌드 |
| 박스를 놓고 마커를 끌었더니 RViz가 꺼짐 | 알려진 현상 — 3-1의 공통 주의사항 참고. 순서를 지키면 발생하지 않는다 |
| Plan이 계속 실패 | 세 갈래 점검 — ① 도달(팔 길이 밖?) ② 자세(무리한 orientation?) ③ 충돌(Scene 장애물과 겹침?) — 목표를 로봇 가까이로 옮겨 성공부터 재현 |
| RViz에 로봇 안 보임 | Fixed Frame `base_link`? MotionPlanning 디스플레이 체크? |
| Setup Assistant에서 저장 폴더를 못 찾음 | 새로 만드는 폴더라 Browse로는 안 나온다 — 경로를 직접 타이핑 (3-4의 11단계) |
| Generate Package를 눌렀는데 폴더가 없음 | Exit를 먼저 눌렀을 가능성 — `find ~/ros2_ws/src -name "rm_75_jaw_config"`로 확인 후 없으면 3-4를 다시 진행 |
| Gazebo 연동에서 실행이 중간에 멈춤 | 3-3의 허용 오차 완화 + 속도 스케일 0.1로 낮춰 재시도 |

---

## 4. 토픽·서비스·액션 다뤄보기 (시뮬레이션 팔)

MoveIt 없이, ROS2의 세 가지 통신 방식으로 팔과 직접 대화해 본다.
**준비**: 앞 절의 창을 모두 닫고 Gazebo만 띄운다.

```bash
pkill -9 -f "gz sim"
ros2 launch rm_gazebo gazebo_75_demo.launch.py
```

> ⚠️ **이 절의 직접 명령은 MoveIt을 거치지 않는다 = 충돌 검사가 없다.**
> 시뮬레이션 전용 실습이며, 실기체에서 이 방식으로 임의 궤적을 보내는 것은 금지다.
> (같은 채널이라도 "읽기"는 언제나 안전하고, "쓰기"는 계획을 거쳐야 한다.)

> 💡 Gazebo로 하는 이유 — demo(mock)로도 명령은 전부 돌아가지만, `effort` 값이나 궤적 추종 오차처럼
> **물리가 있어야 보이는 것**이 나타나지 않는다. 03에서 실기체와 비교하려면 물리 시뮬 쪽이 맞다.

### 4-1. 토픽 — 상태 읽기

```bash
ros2 topic list -t                          # 토픽 목록과 타입
ros2 topic echo /joint_states --once        # 관절 상태 스냅샷 한 장
ros2 topic hz /joint_states                 # 발행 주기 (Ctrl+C로 종료)
ros2 interface show sensor_msgs/msg/JointState   # 메시지 구조
```

- `name` 배열과 `position` 배열이 **같은 순서로 짝지어진다** — `position[3]`이 무슨 관절인지는 `name[3]`을 본다.
- 시뮬레이션의 `effort` 값이 어떤지 봐 두자(0인지, 비어 있는지) — 실기체에서는 전류 기반 추정값이
  들어오며, 정지 상태에도 0이 아니다(중력을 버티는 힘). 03_practice에서 비교한다.

### 4-2. 토픽 — 관절 궤적 직접 발행 (쓰기)

컨트롤러가 구독하는 궤적 토픽을 확인한 뒤 직접 명령을 넣어 본다.

```bash
ros2 topic list | grep trajectory     # /rm_group_controller/joint_trajectory 확인
```

**경유점 1개** (3초에 걸쳐 이동):

```bash
ros2 topic pub --once /rm_group_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7],
  points: [
    { positions: [0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0], time_from_start: {sec: 3} }
  ]
}"
```

**경유점 3개** (시간표 붙은 자세 목록 — "궤적"의 실체):

```bash
ros2 topic pub --once /rm_group_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7],
  points: [
    { positions: [0.5, 0.3, 0.0, 0.8, 0.0, 0.3, 0.0], time_from_start: {sec: 2} },
    { positions: [-0.5, 0.6, 0.0, 1.2, 0.0, 0.6, 0.0], time_from_start: {sec: 5} },
    { positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 8} }
  ]
}"
```

다른 터미널에 `ros2 topic echo /joint_states`를 흘려 두고, 명령한 값으로 `position`이
수렴해 가는 것을 눈으로 좇아 보자.

### 4-3. 액션 — 진행 보고가 있는 긴 작업

궤적 실행처럼 수 초가 걸리는 작업은 액션으로 다룬다 — 목표 전송, **진행 피드백**, 완료/실패
결과, 중간 취소가 가능한 통신 방식이다.

```bash
ros2 action list -t
ros2 interface show control_msgs/action/FollowJointTrajectory   # Goal / Result / Feedback 3부 구조

ros2 action send_goal /rm_group_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7],
    points: [{ positions: [0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0], time_from_start: {sec: 3} }]
  }
}" --feedback
```

`--feedback`으로 실행 중 진행 상황이 스트리밍되는 것이 토픽 pub과의 결정적 차이다.
(MoveIt의 Execute도 내부적으로 이 액션을 쓴다 — 3-5 ②에서 맞춰 준 `action_ns`가 바로 이 이름이다.)

### 4-4. 서비스 — 요청과 응답

```bash
ros2 service list -t
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers "{}"
```

요청을 보내면 즉시 응답 한 번이 돌아오는 방식 — "지금 컨트롤러 목록 줘" 같은 질의에 알맞다.

### 4-5. 왜 좌표(x, y, z) 목표는 여기서 못 주는가

위 방법들은 전부 **관절 각도**만 받는다. "이 좌표로 가라"를 하려면 좌표 → 관절각 변환(역기구학,
IK)이 필요한데, 이 스택에서 IK를 해 주는 층은 **MoveIt뿐**이다. 그래서 픽앤플레이스처럼 작업
공간 좌표로 명령하는 작업은 MoveIt을 거친다 — 3절과 이후 실습의 존재 이유.

### 4-6. 조사 명령 치트시트

```bash
# ── 토픽 ──────────────────────────────
ros2 topic list -t                      # 목록 + 타입
ros2 topic info <토픽> -v               # 발행자/구독자 상세
ros2 interface show <메시지_타입>        # 구조
ros2 topic echo <토픽>                  # 읽기
ros2 topic pub --once <토픽> <타입> '{필드: 값}'   # 쓰기

# ── 서비스 ────────────────────────────
ros2 service list -t
ros2 service type <서비스>
ros2 interface show <서비스_타입>
ros2 service call <서비스> <타입> '{요청_필드: 값}'

# ── 액션 ──────────────────────────────
ros2 action list -t
ros2 action info <액션>
ros2 interface show <액션_타입>
ros2 action send_goal <액션> <타입> '{Goal_필드: 값}' --feedback
```

> 💡 `echo`가 조용한데 `hz`는 뛴다면 **QoS 불일치**를 의심한다 (RELIABLE 구독 vs BEST_EFFORT 발행).
> "안 나온다 = 없다"가 아니다.

시뮬레이션이 손에 붙었으면 → [03_practice](../03_practice/README.md)
