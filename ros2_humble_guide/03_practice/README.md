# 03. 실기체 실습 — 네트워크 설정부터 픽앤플레이스까지 (Humble)

**목표**
1. PC ↔ RM75 이더넷 연결과 드라이버 설정을 완료한다.
2. 실물 팔을 상대로 토픽·서비스·액션을 다루고, 직접 교시(드래그 티칭)를 체험한다.
3. 제조사 기본 모션 명령(MoveJ · MoveJ_P · MoveL)을 실행하고 MoveIt2와의 차이를 눈으로 확인한다.
4. 기본 모션 명령으로 최종 미션 **픽앤플레이스**를 완수한다.

> ⚠️ **모든 명령은 한 줄로 붙여넣는다.** 이 문서의 명령 중 일부는 길어서 여러 줄로 보이지만,
> 터미널에 붙여넣을 때 줄이 끊기면 **메시지 인자 없이 실행되어 모든 필드가 0으로 발행된다.**
> 특히 그리퍼 명령에서 `position: 0`은 **완전히 닫힘**이므로 위험하다.
> 발행 후 `publishing #1:` 줄에 **내가 넣은 값이 그대로 찍혔는지** 매번 확인한다.

> ⚠️ **이 문서는 Jazzy 환경(Ubuntu 24.04)에서 실기체로 검증한 내용을 Humble로 옮긴 것이다.**
> 드라이버의 **토픽 이름·메시지 필드·launch 파일명은 두 브랜치가 동일함을 확인**했으므로
> 명령은 그대로 통한다. 다만 아래는 Humble 실기체에서 아직 확인되지 않았다 —
> 다르게 나오면 멘토에게 공유해 문서를 고친다.
> - `/joint_states`의 발행 주기와 `velocity`/`effort`가 비어 있는지 (2-1절)
> - RViz 종료 현상 (5절)
> - 그리퍼가 열리지 않는 간헐적 오류와 툴 전원 재인가 복구 (6-5절)

---

## 1. 네트워크 설정

구성: `PC (유선) ── 이더넷 케이블 ── RM75 베이스`

### 1-1. PC 유선 IP를 수동으로 설정

우리 실습 로봇의 IP는 **192.168.1.18**로 설정되어 있다. PC를 같은 대역으로 맞춘다.

- Ubuntu 설정 → 네트워크 → 유선 → 톱니바퀴 → IPv4 탭 → **수동(Manual)**
  - 주소: `192.168.1.100` / 넷마스크: `255.255.255.0` / 게이트웨이: 비움
- 적용 후 케이블을 뽑았다 꽂고 확인:

```bash
ip addr show | grep -A2 "state UP"   # 유선 인터페이스에 192.168.1.100 이 붙었는지
ping -c 3 192.168.1.18               # 로봇 전원이 켜져 있으면 응답이 와야 함
```

ping이 안 되면: 케이블·로봇 전원·PC IP 대역(192.168.1.x)·유무선 혼동을 순서대로 점검.

### 1-2. rm_driver 설정 파일 수정

먼저 현재 값을 확인한다:

```bash
grep -nE "arm_ip|udp_ip|udp_cycle|udp_port" \
  ~/ros2_ws/src/ros2_rm_robot/rm_driver/config/rm_75_config.yaml
```

**저장소 기본값은 우리 환경과 다르므로 반드시 수정해야 한다:**

| 키 | 저장소 기본값(humble) | **바꿀 값** | 의미 |
|---|---|---|---|
| `arm_ip` | `"192.168.1.19"` | `"192.168.1.18"` | 로봇 컨트롤러의 IP — 드라이버가 명령을 보낼 주소 |
| `udp_ip` | `"192.168.1.10"` | `"192.168.1.100"` | **PC의 유선 IP** — 로봇이 상태(관절값 등)를 UDP로 밀어 넣을 주소 |

> 💡 기본 IP는 브랜치·버전마다 다르다(jazzy 브랜치는 `192.168.1.17`).
> 그래서 아래 명령은 **어떤 값이 들어 있든** 우리 값으로 바꾸도록 되어 있다.

```bash
cd ~/ros2_ws/src/ros2_rm_robot/rm_driver/config
sed -i -E 's/(arm_ip: )"[0-9.]+"/\1"192.168.1.18"/; s/(udp_ip: )"[0-9.]+"/\1"192.168.1.100"/' rm_75_config.yaml
grep -nE "arm_ip|udp_ip" rm_75_config.yaml
```

(직접 편집하려면 `gedit rm_75_config.yaml`)

> `udp_ip`가 틀리면 명령은 가는데 `/joint_states`가 안 들어오는, 헷갈리는 상태가 된다.
> "상태가 안 보이면 udp_ip부터"를 기억해 두자.

**수정 후 재빌드 필수** — yaml도 빌드해야 install 경로에 반영된다:

```bash
cd ~/ros2_ws
colcon build --packages-select rm_driver
source install/setup.bash

# install 쪽에 반영됐는지 확인 (여기까지 확인해야 안심)
grep -nE "arm_ip|udp_ip" ~/ros2_ws/install/rm_driver/share/rm_driver/config/rm_75_config.yaml
```

### 1-3. 비상정지 수단 확보 (실습 전 필수)

**팔을 움직이기 전에 반드시 확인한다.**

1. **웹 UI를 미리 띄워 둔다** — 브라우저에서 `http://192.168.1.18` 접속, 우측 상단
   **비상정지** 버튼 위치를 확인한다. 이것이 **1차 수단**이다 — 안전 거리에서 즉시 누를 수 있다.
2. **물리 버튼(전원 버튼) 위치를 눈으로 확인**한다 — 로봇 베이스 뒤쪽, 즉 **작업 공간 안**에 있다.
   통신이 끊겨 웹 UI가 듣지 않을 때만 쓰는 **최후 수단**이며, 팔이 움직이는 중에 손을 뻗는 것은 위험하다.
3. **비상정지 전담자**를 정하고, 그 사람은 실습 내내 **웹 UI 화면 앞에** 있는다.

> 웹 UI는 로봇 상태를 보는 창구이기도 하다. 실습 중 이상이 생기면
> `팔: 그리퍼 이상` 같은 경고가 여기에 표시된다.

---

## 2. 실기체 토픽·서비스·액션 다뤄보기

시뮬레이션에서 익힌 조사 루틴을 실물에 그대로 적용한다. 이 절은 **팔을 움직이지 않고 읽기만** 한다.

드라이버만 실행:

```bash
ros2 launch rm_driver rm_75_driver.launch.py
```

정상이면 이런 로그가 나온다 — 기종·컨트롤러 버전·UDP 설정을 **여기서 확인**한다:

```
[rm_driver]: product_version = RM75-BI
[rm_driver]: controller version : 3
[rm_driver]: UDP_Configuration is cycle:5ms,port:8089,...,ip:192.168.1.100,...
```

`ip:`가 1-2에서 넣은 PC IP와 같은지 반드시 확인할 것.

새 터미널에서:

```bash
# ① 토픽 지도 그리기 — rm_driver 네임스페이스의 토픽들이 대거 보인다
ros2 topic list -t
ros2 topic list | grep rm_driver | head -30
```

관찰 포인트: `/rm_driver/xxx_cmd` 와 `/rm_driver/xxx_result` 가 **짝**을 이룬다 —
명령 토픽에 발행하면 결과 토픽으로 성공/실패가 회신되는 구조다. 4절에서 이 구조를 그대로 쓴다.

```bash
# ② 관절 상태 — 실물의 각도가 들어온다
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states

# ③ 메시지 구조 — RealMan 전용 메시지들 구경
ros2 interface list | grep rm_ros_interfaces | head -20

# ④ 노드/서비스/액션
ros2 node list
ros2 param list /rm_driver          # 드라이버 파라미터 (arm_ip 등이 보인다)
ros2 service list -t | head -20
ros2 action list -t
```

### 2-1. 시뮬레이션과 다른 두 가지

**① 상태 보고 주기가 시뮬레이션과 다르다**

`ros2 topic hz /joint_states`를 확인해 보자. 설정이 그대로라면 **200 Hz**가 나온다.
주기를 정하는 주체가 다르기 때문이다:

| | 주기 | 무엇이 정하는가 |
|---|---|---|
| 시뮬레이션 | 100 Hz | `ros2_controllers.yaml`의 `update_rate: 100` |
| **실기체** | **200 Hz** | `rm_75_config.yaml`의 `udp_cycle: 5` (5 ms = 200 Hz) |

`udp_cycle` 값을 직접 확인해 계산이 맞는지 검산해 보자 — `1000 / udp_cycle = Hz`.

시뮬에서는 컨트롤러 매니저가, 실물에서는 **로봇이 UDP로 밀어 넣는 주기**가 상태 보고를 지배한다.

**② `velocity`와 `effort`는 비어 있다 — 정상이다**

```
position: [7개 값 정상]
velocity: []
effort: []
```

RM75 컨트롤러 v3의 드라이버는 `/joint_states`에 **`position`만** 싣는다.
**이 실습에서 관절 토크를 관찰하는 것은 불가능하다** — RM75 기본형에는 관절 토크 센서가 없고,
충돌 보호는 컨트롤러 내부에서 전류를 감시하되 그 값을 ROS로 내보내지 않는다.

관절 속도는 **별도 토픽**으로 나온다:

```bash
ros2 topic echo /rm_driver/udp_joint_speed --once
```

이 값의 단위는 문서화되어 있지 않으므로, **크기보다 "0인가 아닌가"를 관찰 대상으로 삼는다.**

---

## 3. 직접 교시(드래그 티칭) 체험

같은 팔이 모드에 따라 완전히 다른 "성격"을 보이는 것을 몸과 데이터로 확인한다.
**모드 전환은 담당 선배가 티칭 소프트웨어로 수행**하고, 체험은 안내에 따라 진행한다.

| 체험 | 내용 | 관찰 |
|---|---|---|
| ① 위치 제어 상태 | 팔을 **살짝** 밀어 본다 | 버틴다. 무리하게 밀면 충돌 보호가 작동해 정지 — 그것을 확인하는 것까지가 체험 |
| ② 직접 교시 상태 | 팔을 손으로 이끈다 | 부드럽게 따라오고, 놓으면 그 자리에 머문다. 일곱 관절이 고루 순응하는지도 관찰 |
| ③ 재생 (가능 시) | 교시한 자세/궤적을 저장·재생 | 사람의 시연이 로봇의 동작이 되는 과정 (teaching by demonstration의 최소 형태) |

### 3-1. 모니터링 — `echo`로는 볼 수 없다

> ⚠️ **`ros2 topic echo`로는 관찰할 수 없다** — 실기체는 200 Hz로 발행하므로 화면이
> 흘러가 버려 세 국면을 비교할 수 없다. 아래 스크립트로 **0.5초에 한 줄씩** 요약해서 본다.

```bash
mkdir -p ~/rm75_practice
cat > ~/rm75_practice/watch_teaching.py << 'PYEOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rm_ros_interfaces.msg import Jointspeed


class Watch(Node):
    def __init__(self):
        super().__init__('watch_teaching')
        self.pos, self.spd = None, None
        self.create_subscription(JointState, '/joint_states',
                                 lambda m: setattr(self, 'pos', m.position), 10)
        self.create_subscription(Jointspeed, '/rm_driver/udp_joint_speed',
                                 lambda m: setattr(self, 'spd', m.joint_speed), 10)
        self.create_timer(0.5, self.show)

    def show(self):
        if self.pos is None or self.spd is None:
            return
        p = ' '.join(f'{v:+.3f}' for v in self.pos)
        smax = max(abs(v) for v in self.spd)
        print(f'pos[{p}]  |speed|max={smax:.4f}', flush=True)


def main():
    rclpy.init()
    try:
        rclpy.spin(Watch())
    except KeyboardInterrupt:
        pass


main()
PYEOF

python3 ~/rm75_practice/watch_teaching.py
```

세 국면을 비교 기록한다:

| 국면 | `position` | `\|speed\|max` |
|---|---|---|
| 정지 (교시 전) | 소수점 3자리까지 변화 없음 | 0 부근 (0.00 ~ 0.03) |
| 교시 중 | 손을 따라 연속 변화 | **뚜렷하게 비영** — 사람이 준 속도의 그림자 |
| 손을 뗀 직후 | 그 자리에 머묾 | 즉시 0으로 복귀 |

> 나중에 다시 분석하려면 녹화해 두어도 된다:
> ```bash
> ros2 bag record -o teaching_run /joint_states /rm_driver/udp_joint_speed
> ```

> 이 체험이 뒤의 픽앤플레이스에서 실제로 쓰인다 — **홈 자세를 드래그 티칭으로 정한다**(6-2 ④).

---

## 4. 제조사 기본 모션 명령 — MoveJ · MoveJ_P · MoveL

로봇 팔이 "원래 알아듣던 말"이다. 베이스 내장 컨트롤러의 기본 명령을 `rm_driver` 토픽으로 직접
호출해 보고, MoveIt2와의 차이를 눈으로 확인한다.

### 4-1. 개념 (교재 4.1절 요약)

| 명령 | 목표를 주는 방식 | 경로 |
|---|---|---|
| **MoveJ** | 7개 관절의 목표 각도 | 관절 공간 보간 — 손끝은 완만한 **곡선**. 빠르고 무리가 적어 "일단 저기로"의 기본값 |
| **MoveL** | 손끝의 목표 위치·자세 | 작업 공간의 **직선**. 수직 접근·긋기처럼 경로 자체가 중요할 때. 특이점 근처에서 실패·정지 가능 |
| **MoveJ_P** | 손끝의 목표 위치·자세 | 도착점은 데카르트, 이동은 관절식 — 경로보다 **도착점**이 중요할 때 |

세 명령의 공통 성격: ① **결정적**(같은 명령 = 항상 같은 경로) ② **즉시 실행**(호출이 곧 움직임 —
Plan/Execute 분리 없음) ③ **장애물을 모름**(충돌 검사 없음).

> ⚠️ **바로 이 ②③ 때문에 이 절이 가장 위험하다.** 잔상 확인 단계가 없다 = 발행 즉시 팔이 움직인다.
> - 작업 공간을 **완전히 비운다** (테이블 위 물건 포함)
> - `speed`는 **10** (%)으로
> - 웹 UI 비상정지를 띄운 전담자가 자리에 있는지 확인 후 발행
> - **발행 전에 현재 관절값과 목표를 비교**한다:
>   `ros2 topic echo /joint_states --once`로 현재값을 읽고, 목표와의 차이가
>   **0.3 rad(약 17°)를 넘는 관절이 없는지** 본다. 넘으면 현재값에 가깝게 목표를 수정한다.
>   **예시값은 어디까지나 예시이며, 시작 자세에 따라 큰 움직임이 될 수 있다.**

### 4-2. 준비 — 메시지 구조부터 확인

드라이버가 떠 있는 상태(2절)에서, **발행 전에 반드시** 메시지 구조를 확인한다.

```bash
ros2 interface show rm_ros_interfaces/msg/Movej
ros2 interface show rm_ros_interfaces/msg/Movejp
ros2 interface show rm_ros_interfaces/msg/Movel
```

```
# Movej
float32[] joint
uint8 speed
bool block
uint8 trajectory_connect   # 0 = 즉시 계획·실행, 1 = 다음 궤적과 함께 계획 (즉시 실행되지 않음)
uint8 dof
```

> 💡 `trajectory_connect`는 **항상 0**으로 둔다. 1로 두면 명령을 보내도 팔이 움직이지 않아
> "명령이 안 먹는다"고 오해하기 쉽다.

결과 회신을 받을 터미널을 미리 켜 둔다:

```bash
# 터미널 A (결과 대기)
ros2 topic echo /rm_driver/movej_result
```

### 4-3. MoveJ — 관절 각도로 이동

```bash
ros2 topic pub --once /rm_driver/movej_cmd rm_ros_interfaces/msg/Movej "{joint: [0.0, 0.3, 0.0, 0.6, 0.0, 0.6, 0.0], speed: 10, block: true, trajectory_connect: 0, dof: 7}"
```

- 손끝이 **완만한 곡선**을 그리며 이동하는 것을 관찰.
- 터미널 A에 `data: true`가 찍히면 성공.

> 💡 **결정성 확인** — 같은 명령을 그냥 다시 보내면 팔이 이미 그 자리에 있어 움직이지 않는다.
> 결정성을 보려면 **다른 자세로 갔다가 이 명령으로 돌아오기**를 반복한다.
> 몇 번을 반복해도 경로가 똑같다 — MoveIt2와의 결정적 차이다.

### 4-4. 현재 말단 자세 읽기 (MoveL/MoveJ_P 목표 만들기)

임의의 좌표를 지어내지 말고, **현재 자세를 읽어서 조금만 바꾼 목표**를 쓴다 — 도달 불가/이상
자세 사고를 예방하는 요령이다.

```bash
# 터미널 A (결과 대기)
ros2 topic echo /rm_driver/get_current_arm_state_result
```

```bash
# 터미널 B (조회 요청 — 빈 메시지)
ros2 topic pub --once /rm_driver/get_current_arm_state_cmd std_msgs/msg/Empty "{}"
```

터미널 A에 `joint`(7개) · `pose`(위치 m + 자세 쿼터니언) · `err`가 회신된다. **`err: 0`이면 정상.**

```
pose:
  position: {x: 0.3838, y: 0.000003, z: 0.6258}
  orientation: {x: 0.0, y: 0.68127, z: 0.0, w: 0.73203}
```

> ⚠️ **orientation은 그대로 두고 position만 바꾼다.** 자세를 임의로 바꾸면 도달 불가가 되기 쉽다.

### 4-5. MoveJ_P / MoveL — 손끝 자세 목표

**첫 시도는 z만 ±0.05 m** 바꿔 안전하게 한다. 숫자는 반드시 **내가 읽은 현재값** 기반으로 채운다.

```bash
# MoveJ_P — 도착점만 데카르트, 이동은 관절식 (결과: /rm_driver/movej_p_result)
ros2 topic pub --once /rm_driver/movej_p_cmd rm_ros_interfaces/msg/Movejp "{pose: {position: {x: 0.3838, y: 0.0, z: 0.6758}, orientation: {x: 0.0, y: 0.68127, z: 0.0, w: 0.73203}}, speed: 10, trajectory_connect: 0, block: true}"
```

```bash
# MoveL — 같은 두 점을 '직선'으로 (결과: /rm_driver/movel_result)
ros2 topic pub --once /rm_driver/movel_cmd rm_ros_interfaces/msg/Movel "{pose: {position: {x: 0.3838, y: 0.0, z: 0.6258}, orientation: {x: 0.0, y: 0.68127, z: 0.0, w: 0.73203}}, speed: 10, trajectory_connect: 0, block: true}"
```

> 💡 **수직 5 cm로는 두 명령의 경로 차이가 눈에 띄지 않는다.** 차이를 보려면
> `y`를 **±0.1 이상** 바꿔 옆으로 이동시킨다 — MoveJ_P는 살짝 호를 그리고, MoveL은 직선이다.
> 이때도 orientation은 그대로 유지할 것.

- MoveL 실행 중 특이점 근처에서 멈추면 정상적인 보호 동작이다 — 목표를 바꾸거나 MoveJ_P로 대체.
- 참고: `rm_example` 패키지에 같은 명령들의 C++ 예제가 있다 —
  `ros2 pkg executables rm_example`로 목록 확인, 소스(`rm_example/src`)는 코드로 토픽을 발행하는
  좋은 참고 자료다.

### 4-6. 그리퍼 명령 — 세 가지 숫자 (폭·속도·힘)

픽앤플레이스 전에 그리퍼 단독으로 확인해 둔다. **손가락 사이에 아무것도(특히 손) 없는 상태**에서:

```bash
ros2 interface show rm_ros_interfaces/msg/Gripperset
ros2 interface show rm_ros_interfaces/msg/Gripperpick
```

```bash
# 열기 (position 0~1000, 클수록 열림. 1000 = 최대 개방)
ros2 topic pub --once /rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset "{position: 1000, block: true, timeout: 10}"
```

```bash
# 위치 제어로 절반 닫기
ros2 topic pub --once /rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset "{position: 500, block: true, timeout: 10}"
```

```bash
# 힘 제어 파지 — 지정한 힘을 계속 유지 (실제 파지에 쓰는 명령)
ros2 topic pub --once /rm_driver/set_gripper_pick_on_cmd rm_ros_interfaces/msg/Gripperpick "{speed: 200, force: 300, block: true, timeout: 10}"
```

```bash
# 다시 열기
ros2 topic pub --once /rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset "{position: 1000, block: true, timeout: 10}"
```

> **왜 파지에는 pick_on(힘 제어)인가** — 위치 제어로 닫으면 물체에 막힐 때 과부하 보호로 멈추면서
> 힘을 유지하지 않아 이송 중 미끄러진다. `pick_on`은 설정한 힘을 계속 유지한다.
> 힘이 약하면 놓치고, 강하면 물체가 손상된다 — 픽앤플레이스에서 이 값을 튜닝한다.

> ⚠️ **결과 토픽의 `data: true`는 "명령이 접수됐다"는 뜻이지 "의도한 대로 됐다"는 뜻이 아니다.**
> 이미 완전히 닫힌 상태에서 닫기 명령을 또 보내도 `true`가 온다.
> **매번 눈으로 확인**하는 습관을 들일 것. 그리퍼 개방도를 ROS로 읽을 방법은 없다
> (웹 UI의 `설정 → 확장 → 말단 제어 → 그리퍼 → 장치 정보 → 개방도`에서만 볼 수 있다).

### 4-7. 관찰 기록 (교재 6.4절)

한 줄로 남긴다 — "같은 두 점을 잇는 길이 명령마다 달랐다. MoveJ는 ___, MoveL은 ___,
MoveIt2는 ___." 다음 절에서 MoveIt으로 같은 목표를 실행해 세 번째 빈칸을 채운다.

---

## 5. MoveIt2 실습 (실기체)

> ⚠️ **드라이버 중복 실행 금지** — 4절까지 쓰던 `rm_75_driver.launch.py` 터미널을 **Ctrl+C로 종료**한
> 뒤 진행한다. bringup이 드라이버를 포함하고 있어, 중복 실행하면 통신이 꼬인다.

```bash
ros2 launch rm_bringup rm_75_bringup.launch.py
```

드라이버 + MoveIt + RViz가 함께 뜬다. 시뮬레이션에서 하던 것과 화면은 같지만 **Execute가 실물을
움직인다**는 것이 유일하고 결정적인 차이다.

> ⚠️ **실기체에서는 RViz의 Scene Objects를 건드리지 않는다.**
> 박스를 추가한 뒤 **드래그하거나 좌표 스핀 버튼을 누르면 RViz가 종료된다**(실기체에서 특히 빈번).
> 이 절에서는 **Scene을 비운 채로** 마커 이동 → Plan → Execute만 연습한다.
>
> **RViz가 죽어도 팔은 멈추지 않는다.** RViz는 시각화일 뿐이고 실행은 move_group과 드라이버가
> 담당한다. 크래시 시 화면과 Stop 버튼을 잃은 채 팔이 계속 움직일 수 있으므로,
> **웹 UI 비상정지를 항상 띄워 둔다**(1-3절).
>
> RViz가 종료되면 **launch 전체를 다시 실행**한다. RViz만 따로 띄우면 `robot_description`을
> 받지 못해 로봇이 보이지 않는다(`Robot model not loaded`).
>
> (이 현상은 Jazzy에서 확인되었다. Humble에서 발생하지 않더라도 위 순서는 지켜서 나쁠 것이 없다.)

### 5-1. 첫 Plan & Execute

1. RViz MotionPlanning → Planning 탭에서 **Velocity Scaling / Accel Scaling이 0.10**인지 확인
   (설정 파일 값이 반영된 기본값이다. 올리지 말 것)
2. 마커를 현재 자세에서 **가까운 곳**으로 이동 → **Plan**
3. 잔상 애니메이션을 눈으로 확인 → "잔상 확인 — 실행합니다" 콜 → **Execute**
4. 실물 팔이 잔상대로 움직이는지 확인. 4-7의 세 번째 빈칸(MoveIt2는 ___)을 채운다 —
   같은 목표를 여러 번 Plan하면 경로가 달라질 수 있고, 실행 전 계산·확인 단계가 끼어든다.

> 💡 Planning Scene에 등록되지 않은 실물은 플래너에게 보이지 않는다.
> 다만 위 경고대로 **RViz에서 물체를 등록하는 조작은 실기체에서 위험**하므로,
> 이 절에서는 개념만 이해하고 넘어간다. 6절의 픽앤플레이스는 MoveIt을 쓰지 않으므로
> 좌표를 정확히 재는 것이 안전의 전부가 된다.

---

## 6. 픽앤플레이스 실습 — 최종 미션

**미션**: 테이블 위 물병을 집어 올려 홈 자세로 이동한 뒤, 지정한 자리에 내려놓는다.
인지(물체가 어디 있는가)는 이번엔 **사람이 자로 재서** 담당한다 — 이 수동 인지를 카메라가
대신하게 만드는 것이 다음 단계(비전 기반 조작)다.

4절에서 익힌 **기본 모션 명령(MoveJ · MoveJ_P · MoveL)** 만으로 수행한다.
MoveIt을 쓰지 않으므로 **충돌 검사도, 잔상 확인도 없다** — 그만큼 좌표를 정확히 재는 것이
안전의 전부다.

> ⚠️ **이 절은 03에서 가장 위험하다.** 명령을 발행하는 순간 팔이 즉시 움직인다.
> - 작업 공간을 비우고, 물병 외에 아무것도 두지 않는다
> - `speed`는 **10** 고정
> - 웹 UI 비상정지를 띄워 둔 전담자가 자리에 있어야 한다
> - 모든 명령은 **한 줄로** 붙여넣고, `publishing #1:` 값을 매번 확인한다

**성공 기준을 시작 전에 합의**한다 (예: "물병을 놓치지 않고 지정 자리에 세워 놓기 — 3회 중 2회 이상").
애매한 기준은 애매한 회고를 낳는다.

### 6-1. 좌표계 이해 — base_link 기준 +x, +y, +z

모든 측정과 명령은 **로봇 베이스(`base_link`) 기준**이다. 단위는 **미터(m)**.

| 축 | 방향 | 확인 방법 |
|---|---|---|
| **+x** | 로봇 **정면** (팔이 뻗어 나가는 쪽) | 웹 UI 3D 화면의 빨간 축 |
| **+y** | 로봇 기준 **왼쪽** | 초록 축 |
| **+z** | **위쪽** | 파란 축 |

> ⚠️ **단위 혼동이 이 실습 사고 1순위다.** 자로 잰 값은 mm이고 명령은 m다 — `412 mm → 0.412`.
> 0을 하나 빠뜨리면 팔이 10배 먼 곳으로 뻗는다.

**두 사람이 독립적으로 재서 교차 검증**한다 — 로봇의 반복 정밀도는 ±0.05 mm지만 사람의 측정은
수 mm~수 cm 흔들린다. 미션 오차의 대부분은 로봇이 아니라 자에서 나온다.

### 6-2. 실측 4종

#### ① 그리퍼 길이 `L` — Link7 플랜지 면 → 그리퍼 끝

자로 잰다. 대략 **0.12 ~ 0.13 m**.

> **왜 필요한가** — 우리가 눈으로 재는 것은 **그리퍼 끝**의 위치인데,
> `MoveJ_P` / `MoveL` 명령이 받는 것은 **Link7(플랜지)의 위치**다.
> bringup 기본 모델에는 그리퍼가 없어 플랜지가 말단이기 때문이다.
>
> 이번 실습의 자세(`orientation: 0, 0.707, 0, 0.707`)에서는 그리퍼가 **+x 방향으로 뻗어 있으므로**:
>
> ```
> Link7의 x = (그리퍼 끝의 목표 x) − L
> ```
>
> 이 뺄셈을 빠뜨리면 팔이 물병을 `L`만큼 지나쳐 들어간다.

#### ② 접근 위치 — 물병 앞에서 멈출 지점

물병 **앞면에서 5~10 cm 떨어진 곳**을 접근 지점으로 삼는다.
**그리퍼 끝 기준**으로 베이스에서의 거리를 재고, 위 식으로 Link7 좌표로 환산한다.

```
접근_x = (그리퍼 끝이 멈출 x) − L
```

물병이 정면 중앙이면 `y = 0`, 몸통 중앙 높이가 베이스 높이와 비슷하면 `z = 0`이다.
(예: `x = 0.345, y = 0.00, z = 0.00`)

#### ③ 파지 위치 — 물병 앞면까지의 거리

접근 지점에서 물병 앞면까지의 남은 거리 `d`를 잰다. 파지 위치는:

```
파지_x = 접근_x + d
```

(예: 접근 `0.345` + `0.17` = `0.515`)

#### ④ 홈 자세 — 드래그 티칭으로 정한다

숫자로 만들지 말고 **손으로 끌어서** 정한다. 관절이 한계에서 멀고, 팔꿈치가 적당히 굽어 있고,
물병을 든 채로 안정적인 자세면 된다.

1. 담당 선배가 직접 교시 모드로 전환
2. 손으로 원하는 자세까지 팔을 이끈다
3. 교시 모드 해제
4. 관절값을 읽어 기록한다:

```bash
ros2 topic echo /joint_states --once
```

`position` 7개 값이 5단계의 `MoveJ` 목표가 된다.
(예: `[-0.004, 0.317, 0.007, 0.643, -0.011, 0.753, -0.009]`)

### 6-3. 시퀀스 실행

드라이버만 띄운다 (MoveIt은 쓰지 않으므로 bringup은 종료).

```bash
ros2 launch rm_driver rm_75_driver.launch.py
```

**처음에는 한 단계씩** 실행하고, 각 단계가 끝난 뒤 눈으로 확인한 다음 다음 단계로 넘어간다.

#### 1단계 — 물병 앞으로 접근 (MoveJ_P)

도착점만 중요하고 경로는 관절식이어도 되므로 `MoveJ_P`를 쓴다.

```bash
ros2 topic pub --once /rm_driver/movej_p_cmd rm_ros_interfaces/msg/Movejp "{pose: {position: {x: 0.345, y: 0.00, z: 0.00}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}, speed: 10, trajectory_connect: 0, block: true}"
```

- `orientation: (0, 0.707, 0, 0.707)` = y축 90° 회전 → **그리퍼가 정면(+x)을 향하는 측면 파지 자세**.
  물병처럼 세워진 원통은 위에서보다 옆에서 잡는 것이 안정적이다.
- 확인: `/rm_driver/movej_p_result`에 `data: true`, 그리퍼 끝이 물병 앞 예상 위치에 멈췄는가.

> 💡 **첫 시도는 여기서 멈추고 눈으로 정렬을 확인한다.** 좌표 측정 오차를 전진 전에
> 잡아내기 위해서다. 어긋나 있으면 자를 다시 대고 `x`를 mm 단위로 수정해 재시도한다.

#### 2단계 — 그리퍼 최대 개방

```bash
ros2 topic pub --once /rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset "{position: 1000, block: true, timeout: 10}"
```

**실제로 열렸는지 눈으로 확인한다.** `data: true`가 와도 열리지 않는 경우가 있다 (6-5 참고).

#### 3단계 — 직선으로 파지 위치까지 (MoveL)

경로 자체가 중요한 구간이므로 `MoveL`을 쓴다 — 곡선으로 들어가면 물병 옆구리를 치고 들어간다.

```bash
ros2 topic pub --once /rm_driver/movel_cmd rm_ros_interfaces/msg/Movel "{pose: {position: {x: 0.515, y: 0.00, z: 0.00}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}, speed: 10, trajectory_connect: 0, block: true}"
```

- **`orientation`은 1단계와 똑같이 유지한다** — 자세가 바뀌면 직선 접근이 아니게 된다.
- 확인: 물병이 두 손가락 사이에 들어왔는가. 닿아서 밀리고 있으면 즉시 멈추고 `x`를 줄인다.

#### 4단계 — 힘 제어 파지

```bash
ros2 topic pub --once /rm_driver/set_gripper_pick_on_cmd rm_ros_interfaces/msg/Gripperpick "{speed: 200, force: 400, block: true, timeout: 10}"
```

**파지력 튜닝은 이분 탐색으로** — 확실히 놓치는 약한 값과 확실히 쥐는 강한 값을 먼저 찾고
중간으로 좁힌다. (힘 값 / 결과: 미끄러짐·성공·변형) 표를 남기면 그 표가 "힘 감각"의 증거다.

> ⚠️ 파지 성공 여부는 **육안으로 확인**한다. 결과 토픽의 `data: true`로는 알 수 없다.

#### 5단계 — 홈 자세로 이동 (MoveJ)

6-2 ④에서 드래그 티칭으로 정하고 기록한 관절값을 그대로 넣는다.

```bash
ros2 topic pub --once /rm_driver/movej_cmd rm_ros_interfaces/msg/Movej "{joint: [-0.004, 0.317, 0.007, 0.643, -0.011, 0.753, -0.009], speed: 10, block: true, trajectory_connect: 0, dof: 7}"
```

- 관절 공간 보간이라 손끝은 완만한 곡선을 그린다 — **물병을 든 채**이므로 주변 여유를 확인한다.
- 발행 전 4-1의 "0.3 rad 규칙"으로 이동량을 점검한다.

#### 6단계 — 내려놓기 (MoveJ_P → 개방)

물병을 놓을 자리를 정하고, 6-2 ②와 같은 방식으로 좌표를 잰다(그리퍼 끝 기준으로 재고 `L`을 뺀다).
물병 바닥이 테이블에 닿을 높이가 되도록 `z`를 맞춘다.

```bash
# 놓을 자리로 이동 (예: 오른쪽으로 20 cm)
ros2 topic pub --once /rm_driver/movej_p_cmd rm_ros_interfaces/msg/Movejp "{pose: {position: {x: 0.515, y: -0.20, z: 0.00}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}, speed: 10, trajectory_connect: 0, block: true}"
```

```bash
# 그리퍼 열어 놓기
ros2 topic pub --once /rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset "{position: 1000, block: true, timeout: 10}"
```

```bash
# 뒤로 빠지기 (물병에서 손가락을 완전히 뺀다)
ros2 topic pub --once /rm_driver/movel_cmd rm_ros_interfaces/msg/Movel "{pose: {position: {x: 0.345, y: -0.20, z: 0.00}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}, speed: 10, trajectory_connect: 0, block: true}"
```

> ⚠️ **열자마자 팔을 움직이면 물병이 쓰러진다.** 열기 → **물병이 안정된 것을 눈으로 확인** →
> 직선으로 후퇴 순서를 지킨다. 후퇴는 반드시 `MoveL`로 — 곡선으로 빠지면 손가락이 물병을 친다.

### 6-4. 한 번에 이어 실행 — 픽앤플레이스 완성

단계별로 모두 성공했다면, **물병을 원래 자리에 다시 놓고** 1~6단계를 멈춤 없이 연속으로
실행한다. 이것이 미션의 최종 형태다.

연속 실행 시 관찰할 것:

- 각 명령의 `block: true` 덕분에 앞 동작이 끝나야 다음이 시작된다 — **명령을 연달아 붙여도
  겹치지 않는다.**
- 단계 사이에 사람의 판단이 전혀 들어가지 않는다. **이것이 기본 모션 명령의 성격이자 위험이다** —
  좌표가 틀려도 그대로 실행된다.

**시도 사이클(15분)**: 준비 2분 → 실행 5분 → 분석 5분 → **값 하나만** 수정 3분.
좌표와 파지력을 동시에 바꾸면 무엇이 효과였는지 알 수 없다 — 변인 통제.

### 6-5. 실패 모드 도감 — 증상으로 원인 좁히기

| 증상 | 유력 원인 | 1차 처방 |
|---|---|---|
| 그리퍼가 물병 옆을 스치고 지나감 | 좌표 측정 오차 · 축 방향 혼동 | 재측정 + 2인 교차 검증 — 명령보다 **자를 먼저** 의심 |
| 물병을 `L`만큼 지나쳐 들어감 | **그리퍼 길이 `L` 빼기를 누락** | 6-2 ①의 환산식 재확인 |
| 팔이 10배 먼 곳으로 뻗음 | **단위 혼동** (mm를 그대로 입력) | 모든 좌표를 m로 재확인 |
| 명령을 보냈는데 팔이 안 움직임 | `trajectory_connect: 1` | 0으로 수정 (4-2 참고) |
| 전 필드가 0으로 발행됨 | **여러 줄 붙여넣기가 끊김** | `publishing #1:` 값 확인 · 한 줄로 발행 |
| 잡았다가 이송 중 놓침 | 파지력 부족 | `force` 한 단계 ↑ · 파지 후 육안 확인 뒤 다음 단계로 |
| 물병이 찌그러짐 | 파지력 과다 | `force` 한 단계 ↓ |
| 놓을 때 물병이 쓰러짐 | 개방 직후 바로 후퇴 · z가 높아 떨어뜨림 | 열고 나서 잠깐 대기 · `z`를 낮춰 바닥에 닿게 |
| MoveL이 중간에 멈춤 | 특이점 근처 통과 | 정상적인 보호 동작 — 접근 방향을 조금 틀거나 `MoveJ_P`로 대체 |
| 그리퍼가 열리지 않음 (`data: true`인데도) | 그리퍼 드라이버 오류 (간헐적) | 아래 복구 절차 |
| 실기체 보호 정지 | 충돌 보호 (설계된 안전 동작) | 손대지 말고 담당자 콜 · 무엇을 하다 멈췄는지 기록 |
| `/joint_states`가 안 들어옴 | `udp_ip` 오설정 | 1-2절 확인 → `rm_driver` 재빌드 → 재실행 |

#### 그리퍼가 열리지 않을 때 (간헐적)

열기 명령이 `data: true`를 회신하는데도 실제로 열리지 않는 경우가 있다.
웹 UI(`http://192.168.1.18`)에 `팔: 그리퍼 이상 / 드라이버 동작 오류`가 표시되면 이 상태다.
**발생 조건은 특정되지 않았고, 같은 조작을 반복해도 재현되지 않을 때가 있다.**

`clear_system_err`로는 풀리지 않는다(그것은 팔의 오류용). **툴 전원을 재인가한다:**

```bash
ros2 topic pub --once /rm_driver/set_tool_voltage_cmd std_msgs/msg/UInt16 "{data: 0}"
sleep 3
ros2 topic pub --once /rm_driver/set_tool_voltage_cmd std_msgs/msg/UInt16 "{data: 3}"
sleep 3
ros2 topic pub --once /rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset "{position: 1000, block: true, timeout: 10}"
```

⚠️ `data: 3` = **24V**. 웹 UI의 `설정 → 확장 → 말단 제어 → 툴 전원 출력`에서 현재 설정값을
확인하고 같은 값으로 되돌린다. **다른 전압을 넣으면 그리퍼가 손상될 수 있다.**

> 발생하면 **언제 무엇을 하던 중이었는지 기록**해 두자 — 트리거를 좁히는 데 도움이 된다.

### 6-6. 마무리 산출물

- 실행한 명령 전체(측정값이 반영된 최종 좌표·힘 값 포함) + 실행 영상
- **측정 기록표**: 그리퍼 길이 `L` / 접근 x / 파지 x / 놓을 위치 / 홈 관절값 7개
- **파지력 시도 표**: force 값과 결과(미끄러짐 · 성공 · 변형)
- 한 페이지 회고
  ① 좌표를 처음 쟀을 때와 최종값의 차이는 얼마였고, 무엇 때문이었나
  ② 다시 해도 그렇게 할 결정 하나
  ③ 다음 기수에게 조언 한 줄
- 잘 동작한 값들을 **인수인계 노트**로 남긴다.

---

## 7. 다음 단계 예고

이번 미션의 두 가지 한계가 다음 단계의 출발점이다.

**① 인지를 사람이 했다** — 자로 재서 좌표를 입력했다. 이것을 카메라가 대신하면
**비전 기반 조작**이 된다 (RealSense 깊이 카메라 + YOLO 검출 → 물체 좌표 자동 획득).
핸드아이 캘리브레이션 · QoS(BEST_EFFORT) · 좌표 변환(TF)이 새 주제로 등장한다.

**② 충돌 검사가 없었다** — 기본 모션 명령은 장애물을 모른다. 5절에서 다룬 MoveIt2는
Planning Scene에 등록된 물체를 피해 경로를 계산한다. 작업 환경이 복잡해질수록
(테이블, 주변 장비, 다른 물체) MoveIt 기반 시퀀스가 필요해진다.

관심 있는 사람은 선배의 비전 파지(`rm_vision_grasp`) 자료를 참고할 것.
