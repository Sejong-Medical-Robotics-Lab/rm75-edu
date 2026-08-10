"""
수집된 캘리브레이션 데이터의 품질을 진단한다.
- 이미지별 재투영 오차
- 4가지 hand-eye 방법 비교
- 이미지별 체커보드 위치 일관성 (이상치 탐지)
"""

import os

import cv2
import numpy as np
import yaml
import sys

from libs.auxiliary import find_latest_data_folder

np.set_printoptions(precision=4, suppress=True)

current_path = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "eye_hand_data",
)

if len(sys.argv) > 1:
    images_path = os.path.join("eye_hand_data", sys.argv[1])
else:
    images_path = os.path.join(
        "eye_hand_data",
        find_latest_data_folder(current_path),
    )

with open("config.yaml", "r", encoding="utf-8") as file:
    config = yaml.safe_load(file)

args = config.get("checkerboard_args")

XX = args.get("XX")
YY = args.get("YY")
L = args.get("L")

print(f"데이터 폴더: {images_path}")
print(f"체커보드: {XX} x {YY}, 한 칸 {L} m\n")

criteria = (
    cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS,
    30,
    0.001,
)

objp = np.zeros((XX * YY, 3), np.float32)
objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2)
objp = L * objp

obj_points = []
img_points = []
used_index = []

count = len(
    [f for f in os.listdir(images_path) if f.endswith(".jpg")]
)

for i in range(1, count + 1):
    image_file = os.path.join(images_path, f"{i}.jpg")

    if not os.path.exists(image_file):
        continue

    image = cv2.imread(image_file)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    size = gray.shape[::-1]

    found, corners = cv2.findChessboardCorners(
        gray,
        (XX, YY),
        None,
    )

    if found:
        corners2 = cv2.cornerSubPix(
            gray,
            corners,
            (5, 5),
            (-1, -1),
            criteria,
        )

        obj_points.append(objp)
        img_points.append(corners2)
        used_index.append(i)

N = len(img_points)
print(f"검출 성공: {N}/{count}\n")

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points,
    img_points,
    size,
    None,
    None,
)

print("=" * 55)
print("1) 이미지별 재투영 오차 (0.5 px 이하가 정상)")
print("=" * 55)

for index, image_number in enumerate(used_index):
    projected, _ = cv2.projectPoints(
        obj_points[index],
        rvecs[index],
        tvecs[index],
        mtx,
        dist,
    )

    error = cv2.norm(
        img_points[index],
        projected,
        cv2.NORM_L2,
    ) / len(projected)

    flag = "  <-- 나쁨" if error > 0.5 else ""

    distance = np.linalg.norm(tvecs[index])

    print(
        f"  {image_number:2d}.jpg  "
        f"오차 {error:.3f} px  "
        f"거리 {distance:.2f} m{flag}"
    )

print(f"\n  전체 RMS: {ret:.4f} px")

# 로봇 pose 읽기
pose_file = os.path.join(images_path, "poses.txt")

with open(pose_file, "r", encoding="utf-8") as file:
    lines = file.readlines()

values = [
    float(v)
    for line in lines
    for v in line.strip().split(",")
]

poses = [values[i:i + 6] for i in range(0, len(values), 6)]

def rpy_to_matrix(rx, ry, rz):
    """save_poses.py와 동일한 Rz @ Ry @ Rx"""
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)],
    ])

    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)],
    ])

    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1],
    ])

    return Rz @ Ry @ Rx

R_tool = []
t_tool = []

for index in used_index:
    pose = poses[index - 1]

    R_tool.append(
        rpy_to_matrix(pose[3], pose[4], pose[5])
    )

    t_tool.append(
        np.array(pose[0:3]).reshape(3, 1)
    )

methods = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}

print("\n" + "=" * 55)
print("2) hand-eye 방법 비교 (결과가 서로 비슷해야 정상)")
print("=" * 55)

results = {}

for name, method in methods.items():
    R_cam, t_cam = cv2.calibrateHandEye(
        R_tool,
        t_tool,
        rvecs,
        tvecs,
        method=method,
    )

    results[name] = (R_cam, t_cam)

    print(
        f"  {name:12s} t = "
        f"{t_cam[0, 0]:+.4f}, "
        f"{t_cam[1, 0]:+.4f}, "
        f"{t_cam[2, 0]:+.4f} m"
    )

t_all = np.array(
    [t.reshape(3) for _, t in results.values()]
)

spread = (t_all.max(axis=0) - t_all.min(axis=0)) * 1000

print(
    f"\n  방법 간 산포: "
    f"{spread[0]:.1f}, "
    f"{spread[1]:.1f}, "
    f"{spread[2]:.1f} mm"
)

# TSAI 결과로 이미지별 보드 위치 일관성 검사
R_cam, t_cam = results["TSAI"]

T_gripper_cam = np.eye(4)
T_gripper_cam[:3, :3] = R_cam
T_gripper_cam[:3, 3] = t_cam.reshape(3)

print("\n" + "=" * 55)
print("3) 이미지별 체커보드 위치 (모두 같아야 정상)")
print("=" * 55)

board_positions = []

for index in range(N):
    T_base_gripper = np.eye(4)
    T_base_gripper[:3, :3] = R_tool[index]
    T_base_gripper[:3, 3] = t_tool[index].reshape(3)

    R_board, _ = cv2.Rodrigues(rvecs[index])

    T_cam_board = np.eye(4)
    T_cam_board[:3, :3] = R_board
    T_cam_board[:3, 3] = tvecs[index].reshape(3)

    T_base_board = (
        T_base_gripper
        @ T_gripper_cam
        @ T_cam_board
    )

    board_positions.append(T_base_board[:3, 3])

board_positions = np.array(board_positions)
median = np.median(board_positions, axis=0)

errors = np.linalg.norm(
    board_positions - median,
    axis=1,
)

for index, image_number in enumerate(used_index):
    position = board_positions[index]

    flag = ""

    if errors[index] > 0.05:
        flag = "  <-- 이상치"
    elif errors[index] > 0.02:
        flag = "  <-- 의심"

    print(
        f"  {image_number:2d}.jpg  "
        f"{position[0]:+.3f}, "
        f"{position[1]:+.3f}, "
        f"{position[2]:+.3f}  "
        f"오차 {errors[index] * 1000:5.1f} mm{flag}"
    )

print(
    f"\n  중앙값 위치: "
    f"{median[0]:+.3f}, "
    f"{median[1]:+.3f}, "
    f"{median[2]:+.3f} m"
)

print(
    f"  평균 오차: {errors.mean() * 1000:.1f} mm"
)

print(
    f"  최대 오차: {errors.max() * 1000:.1f} mm"
)
