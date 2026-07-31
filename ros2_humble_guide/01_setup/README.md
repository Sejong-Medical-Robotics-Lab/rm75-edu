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
> 선배의 Jazzy 자료를 그대로 복사하면 `-b jazzy`가 따라오니 주의.

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
| `ros-humble-gazebo-ros-pkgs` | **Gazebo Classic 11** — Humble의 기본 물리 시뮬레이터 |
| `ros-humble-gazebo-ros2-control` | Gazebo 안에서 ros2_control 컨트롤러를 돌리는 플러그인 |
| `ros-humble-ros2-control` / `-controllers` | `joint_state_broadcaster`, `joint_trajectory_controller` 등 |
| `joint-state-publisher-gui` | 관절 슬라이더 GUI (모델 확인용) |
| `xacro` | URDF 매크로 처리기 |

> ⚠️ **Jazzy와 다른 부분** — Jazzy는 새 Gazebo(`ros-jazzy-ros-gz`, `ros-jazzy-gz-ros2-control` 계열)를
> 쓰지만 Humble은 **Gazebo Classic**(`ros-humble-gazebo-*` 계열)을 쓴다.
> 선배 자료의 `ros-jazzy-ros-gz-*` 설치 명령을 접두어만 바꿔 실행하면 안 된다 — 위 명령을 그대로 사용할 것.

---

## 4. RealMan 통신 라이브러리 설치

`rm_driver`(실기체 드라이버)가 사용하는 제조사 API 라이브러리(`libapi_cpp.so`)를 시스템에 설치한다.

```bash
cd ~/ros2_ws/src/ros2_rm_robot/rm_driver/lib
sudo bash lib_install.sh
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
- 첫 전체 빌드는 몇 분 걸린다.

---

## 6. 터미널 환경 자동 등록

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

아래 패키지들이 보이면 성공 (버전에 따라 `rm_example`, `rm_control` 등이 추가로 보일 수 있음):

```
rm_75_config        ← rm_moveit2_config 안의 MoveIt 설정
rm_bringup
rm_description
rm_driver
rm_gazebo
rm_ros_interfaces
...
```

각 패키지의 launch 파일 이름이 궁금하면 언제든 아래처럼 직접 확인한다 (이후 문서 전체의 원칙):

```bash
ls ~/ros2_ws/src/ros2_rm_robot/rm_gazebo/launch/
ls ~/ros2_ws/src/ros2_rm_robot/rm_bringup/launch/
```

---

## 트러블슈팅

| 증상 | 확인할 것 |
|---|---|
| 빌드 중 `Package 'rm_ros_interfaces' not found` | 5절의 순서(인터페이스 먼저 빌드 → source → 전체 빌드)를 지켰는가 |
| `ros2 launch` 시 패키지/launch를 못 찾음 | 그 터미널에서 source 했는가 (6절의 bashrc 등록 여부) |
| `sudo rosdep init` 오류 | 이미 초기화된 것 — 무시하고 `rosdep update`만 실행 |
| colcon 빌드가 멈추거나 PC가 느려짐 | 메모리 부족 — `colcon build --executor sequential`로 재시도 |
| RViz/Gazebo가 검게 뜨거나 깨짐 | 그래픽 드라이버 문제. NVIDIA 드라이버 업데이트 직후라면 커널 모듈과 라이브러리 버전 불일치이므로 `sudo reboot` |
| 경로에 한글이 섞여 오류 | 홈 폴더의 `다운로드` 같은 한글 경로를 쓰지 말 것 — 이 가이드는 파일 작업을 `/tmp`와 `~/ros2_ws` 안에서만 한다 |

환경 구축이 끝났으면 → [02_simulation](../02_simulation/README.md)
