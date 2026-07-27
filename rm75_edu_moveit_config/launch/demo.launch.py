"""RM75 교육용 demo 모드 (교재 4.4의 그 명령).

    ros2 launch rm75_edu_moveit_config demo.launch.py

RViz + move_group + 가상 컨트롤러(mock)가 함께 뜬다 — 물리 없는 운동학
시뮬레이션(교재 4.3). RM75 모델과 주황색 마커가 보이면 성공.
"""
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "rm75_edu", package_name="rm75_edu_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
