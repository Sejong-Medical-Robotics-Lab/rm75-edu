"""rm75_edu.gripper_client — 그리퍼의 세 숫자(폭·속도·힘) 창구 (교재 5.3).

demo 모드의 그리퍼는 가상 컨트롤러(gripper_controller, FollowJointTrajectory
액션)로 움직인다 — ros2 action list 에 보이는 그 액션이다(미션 A ⑤).

demo 모드의 한계(교재 7.2가 미리 말해 둔 것): 물리가 없으므로 '파지'는
흉내다 — 손가락은 명령한 폭으로 항상 닫히고, grasp() 는 항상 성공을
보고한다. 힘(force)·속도(speed) 값은 실기체에서 의미를 가지며, demo 에서는
기록·설계 연습용으로만 쓰인다. 실기체의 파지 성공 판정 논리
("목표 폭 전에 힘이 걸려 멈춤 = 쥐었다")는 교재 5.3 참고.
"""
from __future__ import annotations

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .poses import GRIPPER_JOINTS, GRIPPER_WIDTH_MAX

ACTION = "/gripper_controller/follow_joint_trajectory"


class GripperClient:
    def __init__(self, node: Node | None = None):
        self._own = node is None
        if self._own:
            if not rclpy.ok():
                rclpy.init()
            node = rclpy.create_node("gripper_client")
        self.node = node
        self._cli = ActionClient(node, FollowJointTrajectory, ACTION)
        if not self._cli.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                f"{ACTION} 액션 서버가 없습니다 — demo 런치가 먼저 떠 있어야 "
                "합니다.")

    def command(self, width: float, speed: float = 0.05,
                force: float = 5.0) -> bool:
        """폭[m]·속도[m/s]·힘[N] — 세 숫자로 그리퍼를 움직인다.

        demo: 폭→손가락 관절 목표, 속도→이동 시간, 힘→기록만(물리 없음).
        """
        width = min(max(float(width), 0.0), GRIPPER_WIDTH_MAX)
        q = width / 2.0                       # 관절값 = 폭의 절반 (양쪽 대칭)
        dur = max(width / max(speed, 1e-3) if speed > 0 else 0.5, 0.3)

        traj = JointTrajectory()
        traj.joint_names = list(GRIPPER_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [q, q]
        pt.time_from_start = Duration(sec=int(dur),
                                      nanosec=int((dur % 1) * 1e9))
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self._cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf)
        ok = rf.result().result.error_code == 0
        self._last_force = force
        return ok

    def open(self) -> bool:
        return self.command(GRIPPER_WIDTH_MAX, speed=0.05)

    def close(self) -> bool:
        return self.command(0.0, speed=0.05)

    def grasp(self, width: float, speed: float = 0.02,
              force: float = 5.0) -> bool:
        """지정 폭·힘으로 파지 → 성공 여부 보고.

        ⚠ demo 모드에서는 항상 True (물리 없음 — 흉내). 그래도 반환값을
        확인하는 코드 습관을 들일 것: 실기체에서는 이 보고를 건너뛰면
        '허공 배달'(교재 5.3)이 된다.
        """
        ok = self.command(width, speed, force)
        print(f"[gripper] grasp width={width:.3f} m, force={force:.1f} N → "
              f"{'성공' if ok else '실패'} (demo: 파지는 흉내 — 판정은 실기체에서)")
        return ok

    def shutdown(self):
        if self._own:
            self.node.destroy_node()
