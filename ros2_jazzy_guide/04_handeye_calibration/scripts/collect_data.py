# coding=utf-8

import json
import logging
import os
import socket
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs

from libs.log_setting import CommonLog
from libs.auxiliary import (
    create_folder_with_date,
    get_ip,
    popup_message,
)

# ============================================================
# 사용자 설정
# ============================================================

# 실제 체커보드:
# 가로 사각형 9개, 세로 사각형 6개
# OpenCV에는 내부 코너 개수를 입력하므로 8 x 5
CHECKERBOARD_SIZE = (8, 5)

# 화면 표시 배율
# 저장 이미지는 원본 640 x 480으로 저장한다.
DISPLAY_SCALE = 2.0

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# 데이터 저장 폴더 생성
cam0_origin_path = create_folder_with_date()

logger_ = logging.getLogger(__name__)
logger_ = CommonLog(logger_)

count = 1
client = None

CORNER_CRITERIA = (
    cv2.TERM_CRITERIA_EPS
    + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001,
)

def detect_checkerboard(frame):
    """
    영상에서 체커보드 내부 코너를 검출한다.

    반환값:
        found:
            체커보드 검출 성공 여부

        corners:
            정밀화된 코너 좌표
    """
    gray = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2GRAY,
    )

    found, corners = cv2.findChessboardCorners(
        gray,
        CHECKERBOARD_SIZE,
        flags=(
            cv2.CALIB_CB_ADAPTIVE_THRESH
            | cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_FAST_CHECK
        ),
    )

    if not found:
        return False, None

    refined_corners = cv2.cornerSubPix(
        gray,
        corners,
        winSize=(11, 11),
        zeroZone=(-1, -1),
        criteria=CORNER_CRITERIA,
    )

    return True, refined_corners

def draw_status_overlay(
    image,
    checkerboard_found,
    saved_count,
):
    """
    화면에 검출 상태와 저장 개수를 표시한다.
    """
    if checkerboard_found:
        detection_text = "CHECKERBOARD DETECTED"
        detection_color = (0, 255, 0)
    else:
        detection_text = "CHECKERBOARD NOT DETECTED"
        detection_color = (0, 0, 255)

    cv2.putText(
        image,
        detection_text,
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.85,
        detection_color,
        2,
    )

    cv2.putText(
        image,
        f"SAVED SAMPLES: {saved_count}",
        (20, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 255),
        2,
    )

    cv2.putText(
        image,
        "s: save | q: quit",
        (20, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2,
    )

def callback(frame):
    """
    카메라 프레임 표시 및 키 입력 처리.

    반환값:
        True:
            계속 실행

        False:
            종료
    """
    global count
    global client

    checkerboard_found, corners = detect_checkerboard(
        frame
    )

    display_image = frame.copy()

    if checkerboard_found:
        cv2.drawChessboardCorners(
            display_image,
            CHECKERBOARD_SIZE,
            corners,
            checkerboard_found,
        )

    saved_count = count - 1

    draw_status_overlay(
        display_image,
        checkerboard_found,
        saved_count,
    )

    display_image = cv2.resize(
        display_image,
        None,
        fx=DISPLAY_SCALE,
        fy=DISPLAY_SCALE,
        interpolation=cv2.INTER_LINEAR,
    )

    cv2.imshow(
        "Hand-Eye Calibration Data Collection",
        display_image,
    )

    key = cv2.waitKey(1) & 0xFF

    # q를 누르면 종료
    if key == ord("q"):
        logger_.info(
            "사용자 요청으로 데이터 수집을 종료합니다."
        )
        return False

    # s를 누르면 저장
    if key == ord("s"):

        if not checkerboard_found:
            logger_.warning(
                "체커보드가 검출되지 않아 저장하지 않았습니다."
            )
            return True

        socket_command = json.dumps(
            {
                "command": "get_current_arm_state",
            }
        )

        state, pose = send_cmd(
            client,
            socket_command,
        )

        if state:
            logger_.info(
                f"로봇 상태 획득 성공, 현재 자세: {pose}"
            )
        else:
            logger_.error(
                f"로봇 상태 획득 실패: {pose}"
            )
            return True

        pose_file_path = os.path.join(
            cam0_origin_path,
            "poses.txt",
        )

        image_path = os.path.join(
            cam0_origin_path,
            f"{count}.jpg",
        )

        try:
            # 원본 RGB 이미지 저장
            image_saved = cv2.imwrite(
                image_path,
                frame,
            )

            if not image_saved:
                raise RuntimeError(
                    f"이미지 저장 실패: {image_path}"
                )

            # 로봇 pose 저장
            with open(
                pose_file_path,
                "a",
                encoding="utf-8",
            ) as file:
                pose_string = ",".join(
                    str(value)
                    for value in pose
                )

                file.write(
                    f"{pose_string}\n"
                )

            logger_.info(
                f"=== {count}번째 데이터 수집 완료 ==="
            )

            logger_.info(
                f"이미지: {image_path}"
            )

            logger_.info(
                f"로봇 자세: {pose}"
            )

            count += 1

        except Exception as error:
            logger_.error(
                f"데이터 저장 중 오류 발생: {error}"
            )

            # pose 저장 실패 등으로 이미지와 pose 개수가 달라지는 것을 방지
            if os.path.exists(image_path):
                try:
                    os.remove(image_path)

                    logger_.warning(
                        f"불완전한 이미지 삭제: {image_path}"
                    )

                except OSError as remove_error:
                    logger_.error(
                        f"이미지 삭제 실패: {remove_error}"
                    )

    return True

def send_cmd(
    client_socket,
    cmd,
    get_pose=True,
):
    """
    로봇팔에 명령을 전송하고 응답을 받는다.

    get_pose=True이면:
        (성공 여부, pose 또는 오류 메시지)

    get_pose=False이면:
        명령 전송 성공 여부
    """
    try:
        client_socket.sendall(
            cmd.encode("utf-8")
        )

        if not get_pose:
            response = client_socket.recv(
                4096
            ).decode(
                "utf-8",
                errors="replace",
            )

            logger_.info(
                f"로봇 응답: {response}"
            )

            return True

        time.sleep(0.1)

        response = client_socket.recv(
            4096
        ).decode(
            "utf-8",
            errors="replace",
        )

        logger_.info(
            f"로봇 응답: {response}"
        )

        decoder = json.JSONDecoder()
        data_list = []
        index = 0

        # 하나의 응답에 여러 JSON이 포함될 수 있으므로 분리한다.
        while index < len(response):

            while (
                index < len(response)
                and response[index].isspace()
            ):
                index += 1

            if index >= len(response):
                break

            try:
                obj, parsed_length = decoder.raw_decode(
                    response[index:]
                )

                data_list.append(obj)

                index += parsed_length

            except json.JSONDecodeError as error:
                logger_.error(
                    f"JSON 파싱 오류: {error}"
                )
                break

        # current_arm_state 응답을 찾는다.
        target_data = None

        for data in reversed(data_list):
            if (
                isinstance(data, dict)
                and data.get("state")
                == "current_arm_state"
            ):
                target_data = data
                break

        if target_data is None:
            return (
                False,
                "유효한 로봇 상태 응답을 찾지 못했습니다.",
            )

        arm_state = target_data["arm_state"]

        # ====================================================
        # 로봇 오류 처리
        # ====================================================

        arm_error_raw = arm_state.get(
            "err",
            [],
        )

        try:
            arm_errors = [
                int(error_code)
                for error_code in arm_error_raw
            ]

        except (TypeError, ValueError):
            return (
                False,
                f"알 수 없는 오류 형식: {arm_error_raw}",
            )

        # 0은 정상
        # 8193은 현재 환경에서 그리퍼 관련 오류로 발생 중이므로
        # 캘리브레이션 pose 저장에는 영향을 주지 않도록 경고만 출력
        ignorable_errors = {
            0,
            8193,
        }

        unexpected_errors = [
            error_code
            for error_code in arm_errors
            if error_code not in ignorable_errors
        ]

        if unexpected_errors:
            return (
                False,
                f"로봇 오류 발생: {unexpected_errors}",
            )

        if 8193 in arm_errors:
            logger_.warning(
                "오류 코드 8193이 감지됐지만, "
                "말단 pose 데이터가 포함되어 있어 "
                "캘리브레이션 수집을 계속합니다."
            )

        # ====================================================
        # 로봇 pose 변환
        # ====================================================

        pose_raw = arm_state["pose"]

        if len(pose_raw) != 6:
            return (
                False,
                f"잘못된 pose 길이: {len(pose_raw)}",
            )

        pose_converted = [
            pose_raw[0] / 1_000_000,
            # x: 0.001 mm → m

            pose_raw[1] / 1_000_000,
            # y: 0.001 mm → m

            pose_raw[2] / 1_000_000,
            # z: 0.001 mm → m

            pose_raw[3] / 1_000,
            # rx: 0.001 rad → rad

            pose_raw[4] / 1_000,
            # ry: 0.001 rad → rad

            pose_raw[5] / 1_000,
            # rz: 0.001 rad → rad
        ]

        return True, pose_converted

    except json.JSONDecodeError:
        return (
            False,
            "JSON 파싱 오류",
        )

    except KeyError as error:
        return (
            False,
            f"응답에 필요한 필드가 없습니다: {error}",
        )

    except socket.timeout:
        return (
            False,
            "로봇 응답 시간 초과",
        )

    except OSError as error:
        return (
            False,
            f"소켓 통신 오류: {error}",
        )

    except Exception as error:
        return (
            False,
            f"응답 처리 중 오류 발생: {error}",
        )

def display_d435():
    """
    RealSense D435 영상을 표시하고
    캘리브레이션 데이터를 수집한다.
    """
    global count

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(
        rs.stream.color,
        CAMERA_WIDTH,
        CAMERA_HEIGHT,
        rs.format.bgr8,
        CAMERA_FPS,
    )

    try:
        pipeline.start(config)

    except Exception as error:
        logger_.error(
            f"카메라 연결 오류: {error}"
        )

        popup_message(
            "알림",
            "카메라 연결에 실패했습니다.",
        )

        sys.exit(1)

    count = 1

    logger_.info(
        "손눈 캘리브레이션 데이터 수집을 시작합니다."
    )

    logger_.info(
        f"체커보드 내부 코너: "
        f"{CHECKERBOARD_SIZE[0]} x "
        f"{CHECKERBOARD_SIZE[1]}"
    )

    logger_.info(
        f"데이터 저장 폴더: {cam0_origin_path}"
    )

    logger_.info(
        "체커보드가 검출된 상태에서 s를 누르세요."
    )

    logger_.info(
        "종료하려면 q를 누르세요."
    )

    try:
        while True:
            frames = pipeline.wait_for_frames()

            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            color_image = np.asanyarray(
                color_frame.get_data()
            )

            should_continue = callback(
                color_image
            )

            if not should_continue:
                break

    except KeyboardInterrupt:
        logger_.info(
            "Ctrl+C가 입력되어 종료합니다."
        )

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

        logger_.info(
            f"최종 수집 개수: {count - 1}"
        )

        logger_.info(
            f"저장 폴더: {cam0_origin_path}"
        )

def main():
    """
    로봇에 연결하고 작업 좌표계를 Base로 설정한 뒤
    데이터 수집을 시작한다.
    """
    global client

    robot_ip = get_ip()

    logger_.info(
        f"로봇 IP: {robot_ip}"
    )

    if not robot_ip:
        popup_message(
            "알림",
            "로봇 IP에 ping 응답이 없습니다.",
        )

        sys.exit(1)

    try:
        client = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM,
        )

        client.settimeout(3.0)

        client.connect(
            (
                robot_ip,
                8080,
            )
        )

        logger_.info(
            f"로봇 연결 성공: {robot_ip}:8080"
        )

        # 작업 좌표계를 Base로 설정
        socket_command = json.dumps(
            {
                "command": "set_change_work_frame",
                "frame_name": "Base",
            }
        )

        work_frame_result = send_cmd(
            client,
            socket_command,
            get_pose=False,
        )

        if not work_frame_result:
            logger_.warning(
                "Base 작업 좌표계 설정 응답을 확인하지 못했습니다."
            )

        display_d435()

    except ConnectionRefusedError:
        logger_.error(
            "로봇 연결이 거부되었습니다."
        )

        popup_message(
            "알림",
            "로봇에 연결할 수 없습니다.",
        )

        sys.exit(1)

    except socket.timeout:
        logger_.error(
            "로봇 연결 시간이 초과되었습니다."
        )

        popup_message(
            "알림",
            "로봇 연결 시간이 초과되었습니다.",
        )

        sys.exit(1)

    except Exception as error:
        logger_.error(
            f"프로그램 실행 중 오류 발생: {error}"
        )

        popup_message(
            "알림",
            f"오류가 발생했습니다.\n{error}",
        )

        sys.exit(1)

    finally:
        if client is not None:
            try:
                client.close()

                logger_.info(
                    "로봇 소켓 연결을 종료했습니다."
                )

            except OSError:
                pass

if __name__ == "__main__":
    main()
