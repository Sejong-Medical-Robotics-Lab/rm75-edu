"""
compute_in_hand.py의 결과로부터 static_transform_publisher 명령을 만든다.

값을 손으로 옮겨 적지 않는다 — 재계산 후 갱신을 잊어 옛 값으로 검증하는 사고를
막기 위해서다. calibration_result_*.txt를 직접 읽는다.

사용법:
    # 1) 계산 결과를 파일로 남기고
    python3 compute_in_hand.py | tee calibration_result_0810.txt

    # 2) 그 파일로 명령 생성 (인자 생략 시 가장 최근 파일 자동 선택)
    python3 make_tf_cmd.py
    python3 make_tf_cmd.py calibration_result_0810.txt
"""

import glob
import os
import re
import sys

import numpy as np
from scipy.spatial.transform import Rotation

TOOL_FRAME = "Link7"
CAMERA_FRAME = "camera_color_optical_frame"

# 이상적인 장착 자세와 비교해 타당성을 점검한다.
# 우리 브래킷은 z축 -90도 회전에 가깝다. 다른 장착이면 이 값을 바꾼다.
EXPECTED_ROTATION = ("z", -90)

# 이 각도를 넘으면 경고한다.
ROTATION_WARN_DEG = 5.0


def find_result_file():
    """가장 최근에 수정된 calibration_result_*.txt를 찾는다."""
    candidates = glob.glob("calibration_result_*.txt")

    if not candidates:
        raise FileNotFoundError(
            "calibration_result_*.txt 를 찾을 수 없습니다.\n"
            "먼저 실행하세요: "
            "python3 compute_in_hand.py | tee calibration_result_$(date +%m%d).txt"
        )

    return max(candidates, key=os.path.getmtime)


def parse_result(path):
    """
    compute_in_hand.py 출력에서 회전행렬과 병진벡터를 읽는다.

    반환값:
        (3x3 회전행렬, 길이 3 병진벡터)
    """
    text = open(path, encoding="utf-8").read()

    # 로그 형식:
    #   ... INFO: 회전행렬:
    #    [[ ... ]
    #     [ ... ]
    #     [ ... ]]
    #   ... INFO: 병진벡터:
    #    [[ ... ]
    #     [ ... ]
    #     [ ... ]]
    numbers = {}

    for key, count in (("회전행렬", 9), ("병진벡터", 3)):
        match = re.search(
            key + r":\s*\n(.*?)\]\]",
            text,
            re.DOTALL,
        )

        if match is None:
            raise ValueError(
                f"{path} 에서 '{key}' 를 찾지 못했습니다. "
                "compute_in_hand.py 의 출력이 맞는지 확인하세요."
            )

        found = re.findall(
            r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?",
            match.group(1),
        )

        if len(found) < count:
            raise ValueError(
                f"'{key}' 의 숫자 개수가 부족합니다: "
                f"{len(found)}개 (기대 {count}개)"
            )

        numbers[key] = [float(v) for v in found[:count]]

    rotation_matrix = np.array(numbers["회전행렬"]).reshape(3, 3)
    translation = np.array(numbers["병진벡터"])

    return rotation_matrix, translation


def sanity_check(rotation_matrix, translation):
    """계산 결과가 물리적으로 타당한지 점검하고 경고를 출력한다."""
    warnings = []

    # 정규직교 행렬인지
    orthogonality = np.abs(
        rotation_matrix @ rotation_matrix.T - np.eye(3)
    ).max()

    if orthogonality > 1e-4:
        warnings.append(
            f"회전행렬이 정규직교가 아닙니다 (편차 {orthogonality:.2e}). "
            "값을 잘못 읽었을 수 있습니다."
        )

    # 장착 설계상 기대되는 회전과의 차이
    axis, degrees = EXPECTED_ROTATION

    expected = Rotation.from_euler(
        axis,
        degrees,
        degrees=True,
    ).as_matrix()

    difference_deg = np.degrees(
        np.linalg.norm(
            Rotation.from_matrix(
                rotation_matrix.T @ expected
            ).as_rotvec()
        )
    )

    print(
        f"  기대 장착 자세({axis}축 {degrees}도)와의 차이: "
        f"{difference_deg:.2f} deg"
    )

    if difference_deg > ROTATION_WARN_DEG:
        warnings.append(
            f"기대 장착 자세와 {difference_deg:.1f}도 차이가 납니다. "
            "데이터 수집(촬영 거리·자세 다양성)을 점검하세요."
        )

    # 병진 크기
    distance = np.linalg.norm(translation)

    print(f"  카메라까지의 거리: {distance * 1000:.1f} mm")

    if distance > 0.30:
        warnings.append(
            f"플랜지에서 카메라까지 {distance * 1000:.0f} mm 로 계산됐습니다. "
            "자로 잰 실제 거리와 크게 다르면 결과를 신뢰하지 마세요."
        )

    return warnings


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else find_result_file()

    print(f"결과 파일: {path}\n")

    rotation_matrix, translation = parse_result(path)

    print("[타당성 점검]")
    warnings = sanity_check(rotation_matrix, translation)

    for message in warnings:
        print(f"  [경고] {message}")

    if not warnings:
        print("  이상 없음")

    quaternion = Rotation.from_matrix(rotation_matrix).as_quat()

    print("\n[static TF 명령] 아래를 복사해 별도 터미널에서 실행하세요")
    print("-" * 70)
    print("ros2 run tf2_ros static_transform_publisher \\")

    print(
        f"  --x {translation[0]:.6f} "
        f"--y {translation[1]:.6f} "
        f"--z {translation[2]:.6f} \\"
    )

    print(
        f"  --qx {quaternion[0]:.6f} "
        f"--qy {quaternion[1]:.6f} "
        f"--qz {quaternion[2]:.6f} "
        f"--qw {quaternion[3]:.6f} \\"
    )

    print(f"  --frame-id {TOOL_FRAME} --child-frame-id {CAMERA_FRAME}")
    print("-" * 70)

    print("\n[05에서 쓸 값]")
    print(f"  translation = {np.round(translation, 6).tolist()}")
    print(f"  quaternion  = {np.round(quaternion, 6).tolist()}   # x, y, z, w")


if __name__ == "__main__":
    main()
