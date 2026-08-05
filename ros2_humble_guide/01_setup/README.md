# 01. 환경 구축 — Ubuntu 22.04 + ROS2 Humble

**목표**: RM75 실습용 ROS2 워크스페이스를 만들고, RealMan 공식 저장소(humble 브랜치)를 빌드한다.
소요 시간: 약 30분~1시간 (네트워크 속도에 따라 다름)

---

## 0. 사전 확인

```bash
lsb_release -a          # Ubuntu 22.04 인지 확인
printenv ROS_DISTRO     # humble 이 출력되어야 함
```

`ROS_DISTRO`가 비어 있으면 `source /opt/ros/humble/setup.bash` 후 다시 확인.
ROS2 자체가 설치되어 있지 않다면 아래를 펼쳐서 먼저 설치한다.

<details>
<summary><b>ROS2 Humble 설치 (미설치인 경우만)</b></summary>

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
source /opt/ros/humble/setup.bash
```

자세한 내용은 [공식 설치 문서](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 참조.
</details>

---

## 1. 기본 도구 설치

```bash
sudo apt update
sudo apt install -y git unzip python3-pip \
  python3-colcon-common-extensions python3-rosdep liburdfdom-tools
sudo rosdep init 2>/dev/null || true    # 이미 초기화된 PC라면 오류가 나도 무시
rosdep update
```

- `liburdfdom-tools` : URDF 검증용 `check_urdf` 명령 (02 시뮬레이션에서 사용)
- `unzip` : 그리퍼 모델 압축 해제용

---

## 2. 워크스페이스 생성 + 저장소 클론

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b humble https://github.com/RealManRobot/ros2_rm_robot.git
```

> ⚠️ **반드시 `-b humble`** — 이 저장소는 ROS2 배포판별로 브랜치가 나뉘어 있다
> (브랜치 목록은 GitHub 페이지 좌측 상단에서 확인 가능).
> 브랜치를 지정하지 않으면 기본 브랜치가 받아지는데, 그것이 humble이라는 보장이 없다.

클론이 끝나면 브랜치를 확인한다:

```bash
git -C ~/ros2_ws/src/ros2_rm_robot branch --show-current   # humble 이 나와야 함
```

> 💡 저장소에 포함된 `rm_install/scripts/ros2_install.sh`, `moveit2_install.sh`는
> **ROS2와 MoveIt2 자체를 설치해 주는 스크립트**다. 우리는 이미 Humble이 설치되어 있고
> 아래 3절에서 apt로 직접 설치하므로, 이 두 스크립트는 **실행하지 않는다**.
> (단, 4절의 `lib_install.sh`는 반드시 실행한다.)

---

## 3. 의존 패키지 설치

```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro

# 저장소가 요구하는 나머지 의존성 자동 설치
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

| 패키지 | 용도 |
|---|---|
| `ros-humble-moveit` | 모션 플래닝 전체 (RViz 플러그인, Setup Assistant 포함) |
| `ros-humble-gazebo-ros-pkgs` | **Gazebo Classic** + ROS 브리지 — Humble의 기본 물리 시뮬레이터 |
| `ros-humble-gazebo-ros2-control` | Gazebo Classic 안에서 ros2_control 컨트롤러를 돌리는 플러그인 |
| `ros-humble-ros2-control` / `-controllers` | `joint_state_broadcaster`, `joint_trajectory_controller` 등 |
| `joint-state-publisher-gui` | 관절 슬라이더 GUI (모델 확인용) |
| `xacro` | URDF 매크로 처리기 |

> ⚠️ **Jazzy(작성자 리허설 환경)와 다른 부분** — Jazzy는 새 Gazebo(`ros-jazzy-ros-gz`,
> `ros-jazzy-gz-ros2-control`)를 쓰지만 Humble은 **Gazebo Classic**
> (`ros-humble-gazebo-ros-pkgs`, `ros-humble-gazebo-ros2-control`)을 쓴다.
> 인터넷에서 찾은 새 Gazebo용 명령(`ros-gz` 계열)을 접두어만 바꿔 실행하면 안 된다 — 위 명령을 그대로 사용할 것.
>
> 💡 22.04에서는 `pip install`이 그대로 동작한다 — 24.04처럼 `--break-system-packages`
> 옵션이 필요 없다 (이 가이드 자체는 pip 설치가 필요 없다).
>
> ⚠️ **`warehouse_ros_mongo` ERROR는 무시한다** — `rosdep install`이 아래처럼
> 빨간 ERROR를 여러 개(`rm_75_config`, `rm_gazebo` 등) 쏟아낼 수 있지만 **정상이다.**
>
> ```
> ERROR: the following packages/stacks could not have their rosdep keys resolved
> rm_75_config: Cannot locate rosdep definition for [warehouse_ros_mongo]
> ...
> Continuing to install resolvable dependencies...
> #All required rosdeps installed successfully   ← 이 줄이 나오면 성공
> ```
>
> 마지막 줄 `#All required rosdeps installed successfully`만 확인하면 된다.
> MoveIt이 계획 결과를 MongoDB에 저장하는 **선택 기능**인데 apt에 없는
> 패키지라 rosdep이 키를 찾지 못하는 것이고, 이번 실습에서는 쓰지 않는다.
> (MoveIt Setup Assistant가 config 패키지를 생성할 때 자동으로 넣는 항목이라
> RealMan 저장소의 모든 기종 config에 들어 있다.)

---

## 4. RealMan 통신 라이브러리 설치

`rm_driver`(실기체 드라이버)가 사용하는 제조사 API 라이브러리(`libapi_cpp.so`)를 시스템에 설치한다.

```bash
cd ~/ros2_ws/src/ros2_rm_robot/rm_driver/lib
sudo bash lib_install.sh
```

마지막에 초록색 `[SUCCESS]`가 나오면 성공. 확인하려면:

```bash
ldconfig -p | grep api_cpp    # libapi_cpp.so => /usr/local/lib/... 가 보여야 함
```

---

## 5. 워크스페이스 빌드

**메시지 패키지(`rm_ros_interfaces`)를 반드시 먼저 빌드**한다 — 다른 패키지들이 이 메시지 정의를
찾아야 컴파일되기 때문이다.

```bash
cd ~/ros2_ws
colcon build --packages-select rm_ros_interfaces
source install/setup.bash
colcon build
source install/setup.bash
```

- 노란색 경고(`SetuptoolsDeprecationWarning` 등)는 무시해도 된다. **빨간색 오류**만 확인한다.
- 마지막에 `Summary: NN packages finished`가 나오고 `failed`가 없으면 성공이다.
  (`N package had stderr output`은 노란 경고가 있었다는 뜻일 뿐 — 실패가 아니다.)
- 첫 전체 빌드는 몇 분 걸린다.

---

## 6. 터미널 환경 자동 등록

> 💡 **이미 등록해 둔 적이 있다면 이 절은 건너뛴다.** 아래 명령은 `>>`로 줄을
> 덧붙이므로 두 번 실행하면 같은 줄이 중복된다(동작에는 문제없지만 지저분하다).
> 먼저 확인하는 습관을 들이자:
>
> ```bash
> grep -n 'setup.bash' ~/.bashrc
> ```

```bash
echo "source /opt/ros/humble/setup.bash"   >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> "launch 파일을 찾을 수 없음(Package not found)" 오류의 대부분은 **새 터미널에서 source 누락**이다.
> `~/.bashrc`에 등록해 두면 이후 모든 터미널에서 자동 적용된다.

---

## 7. 설치 검증

```bash
ros2 pkg list | grep rm_
```

아래처럼 20개 안팎의 패키지가 보이면 성공이다.

```
rm_63_config, rm_65_config, rm_75_config   ← 기종별 MoveIt 설정 (우리 기체는 rm_75_config)
rm_eco62_config, rm_eco63_config, rm_eco65_config
rm_gen72_config, rm_rx75_config            ← 저장소에 포함된 다른 기종용 (무시)
rm_bringup, rm_description, rm_driver
rm_gazebo, rm_moveit2, rm_ros_interfaces
rm_control, rm_example, rm_doc, rm_install
...
```

> 💡 **config가 여러 개 나오는 것이 정상이다** — RealMan 저장소 하나가 전 기종을
> 담고 있기 때문이다. 우리가 쓸 것은 `rm_75_config` 뿐이고 나머지는 건드리지 않는다.
> (정확한 개수와 목록은 브랜치 버전에 따라 다를 수 있다.)
>
> 💡 예제 노드는 이름에 `rm_`이 없어 위 grep에 걸리지 않는다. 함께 보려면:
>
> ```bash
> ros2 pkg list | grep -E 'rm_|arm_|_control'
> # control_arm_move, get_arm_state, force_position_control 등이 추가로 보인다
> ```

각 패키지의 launch 파일 이름이 궁금하면 언제든 아래처럼 직접 확인한다 (이후 문서 전체의 원칙):

```bash
ls ~/ros2_ws/src/ros2_rm_robot/rm_gazebo/launch/
ls ~/ros2_ws/src/ros2_rm_robot/rm_bringup/launch/
ls ~/ros2_ws/src/ros2_rm_robot/rm_moveit2_config/rm_75_config/launch/
```

파일이 수십 개 쏟아지는데, 대부분 다른 기종용이다. 이름 규칙만 알면 된다.

| 규칙 | 의미 |
|---|---|
| `_63_`, `_65_`, `_75_`, `_eco*`, `_gen72_`, `_rx75_` | 기종 — **우리는 `_75_`** |
| `_6f_`, `_6fb_` | 6축 힘센서 장착형(RM75-6F) 전용 — **우리 기체는 접미사 없는 쪽** |

즉 이번 실습에서 쓸 것은 **`75`가 들어가고 `6f`/`6fb`가 없는 파일**이다
(예: `demo.launch.py`, `rm_75_bringup.launch.py`). 어느 단계에서 어느 파일을 쓰는지는
02·03 문서에서 그때그때 안내한다 — 여기서는 위 `ls`로 이름을 눈에 익혀 두기만 하면 된다.

---

## 트러블슈팅

| 증상 | 확인할 것 |
|---|---|
| 빌드 중 `Package 'rm_ros_interfaces' not found` | 5절의 순서(인터페이스 먼저 빌드 → source → 전체 빌드)를 지켰는가 |
| `ros2 launch` 시 패키지/launch를 못 찾음 | 그 터미널에서 source 했는가 (6절의 bashrc 등록 여부) |
| `sudo rosdep init` 오류 | 이미 초기화된 것 — 무시하고 `rosdep update`만 실행 |
| `rosdep install`에서 빨간 ERROR | `warehouse_ros_mongo` 관련이면 무시 (3절 참고) — 마지막 줄이 successfully면 성공 |
| colcon 빌드가 멈추거나 PC가 느려짐 | 메모리 부족 — `colcon build --executor sequential`로 재시도 |
| `lib_install.sh`에서 `not found` 오류 | 저장소 업데이트로 라이브러리 폴더 이름이 바뀐 경우 — `ls`로 실제 폴더명 확인 후 멘토에게 공유 |
| RViz/Gazebo가 검게 뜨거나 깨짐 | 그래픽 드라이버 문제. NVIDIA 드라이버 업데이트 직후라면 커널 모듈과 라이브러리 버전 불일치이므로 `sudo reboot` |
| 경로에 한글이 섞여 오류 | 홈 폴더의 `다운로드` 같은 한글 경로를 쓰지 말 것 — 이 가이드는 파일 작업을 `/tmp`와 `~/ros2_ws` 안에서만 한다 |

환경 구축이 끝났으면 → [02_simulation](../02_simulation/README.md)
