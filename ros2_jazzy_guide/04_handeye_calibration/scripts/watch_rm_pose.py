import time

from Robotic_Arm.rm_robot_interface import (
    RoboticArm,
    rm_thread_mode_e,
)

ROBOT_IP = "192.168.1.18"
ROBOT_PORT = 8080
INTERVAL_SEC = 0.2

def main():
    arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)

    try:
        handle = arm.rm_create_robot_arm(
            ROBOT_IP,
            ROBOT_PORT,
        )

        if handle.id < 0:
            raise RuntimeError("로봇 연결 실패")

        print("RM75 말단 자세 실시간 확인")
        print("종료: Ctrl+C")

        while True:
            code, state = arm.rm_get_current_arm_state()

            if code == 0:
                pose = state["pose"]
                joints = state["joint"]

                print(
                    "\r"
                    f"XYZ = "
                    f"{pose[0]:+.4f}, "
                    f"{pose[1]:+.4f}, "
                    f"{pose[2]:+.4f} m | "
                    f"RPY = "
                    f"{pose[3]:+.3f}, "
                    f"{pose[4]:+.3f}, "
                    f"{pose[5]:+.3f} rad | "
                    f"J7 = {joints[6]:+.2f} deg",
                    end="",
                    flush=True,
                )
            else:
                print(
                    f"\r상태 조회 실패: {code}",
                    end="",
                    flush=True,
                )

            time.sleep(INTERVAL_SEC)

    except KeyboardInterrupt:
        print("\n종료")

    finally:
        arm.rm_delete_robot_arm()

if __name__ == "__main__":
    main()
