"""
자세를 바꿔가며 base_link -> checkerboard 값을 수집하고
산포를 계산해 캘리브레이션 정확도를 평가한다.

TF 버퍼를 백그라운드 스레드에서 계속 갱신하고,
읽어온 값이 충분히 최신인지 검사한다.
"""

import threading

import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from tf2_ros import Buffer, TransformListener

# 이보다 오래된 TF는 신뢰하지 않는다.
MAX_AGE_SEC = 1.0

class HandeyeVerifier(Node):

    def __init__(self):
        super().__init__("handeye_verifier")

        self.buffer = Buffer()

        self.listener = TransformListener(
            self.buffer,
            self,
        )

    def get_position(self):
        """
        최신 base_link -> checkerboard 위치를 돌려준다.

        반환값:
            (위치 배열, TF 나이(초))
        """
        transform = self.buffer.lookup_transform(
            "base_link",
            "checkerboard",
            rclpy.time.Time(),
        )

        stamp = transform.header.stamp

        stamp_sec = (
            stamp.sec + stamp.nanosec * 1e-9
        )

        now_sec = (
            self.get_clock().now().nanoseconds * 1e-9
        )

        age = now_sec - stamp_sec

        translation = transform.transform.translation

        position = np.array(
            [
                translation.x,
                translation.y,
                translation.z,
            ]
        )

        return position, age

def main():
    rclpy.init()

    node = HandeyeVerifier()

    # 백그라운드에서 계속 spin 해야 TF 버퍼가 갱신된다.
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(
        target=executor.spin,
        daemon=True,
    )

    spin_thread.start()

    # 버퍼가 채워질 시간을 준다.
    import time
    time.sleep(2.0)

    samples = []

    print(
        "팔을 옮긴 뒤 Enter를 누르세요. 종료: q + Enter"
    )

    try:
        while True:
            key = input(
                f"\n[{len(samples) + 1}번째] "
                f"Enter=측정, q=종료: "
            )

            if key.strip().lower() == "q":
                break

            try:
                position, age = node.get_position()

            except Exception as error:
                print(f"  측정 실패: {error}")
                continue

            if age > MAX_AGE_SEC:
                print(
                    f"  TF가 {age:.1f}초 전 값입니다. "
                    f"체커보드 검출 상태를 확인하세요."
                )
                continue

            # 직전 값과 동일하면 갱신되지 않은 것이다.
            if samples:
                delta = np.linalg.norm(
                    position - samples[-1]
                )

                if delta < 1e-6:
                    print(
                        "  직전과 동일한 값입니다. "
                        "TF가 갱신되지 않았습니다."
                    )
                    continue

            samples.append(position)

            print(
                f"  X={position[0]:+.4f} "
                f"Y={position[1]:+.4f} "
                f"Z={position[2]:+.4f} m  "
                f"(TF 나이 {age * 1000:.0f} ms)"
            )

    except KeyboardInterrupt:
        pass

    if len(samples) >= 2:
        data = np.array(samples)

        mean = data.mean(axis=0)
        spread = data.max(axis=0) - data.min(axis=0)

        errors = np.linalg.norm(
            data - mean,
            axis=1,
        )

        print("\n" + "=" * 50)
        print(f"측정 개수: {len(samples)}")

        print(
            f"평균 위치: "
            f"{mean[0]:+.4f}, "
            f"{mean[1]:+.4f}, "
            f"{mean[2]:+.4f} m"
        )

        print(
            f"축별 산포: "
            f"X={spread[0] * 1000:.1f} "
            f"Y={spread[1] * 1000:.1f} "
            f"Z={spread[2] * 1000:.1f} mm"
        )

        print(
            f"평균 오차: {errors.mean() * 1000:.1f} mm"
        )

        print(
            f"최대 오차: {errors.max() * 1000:.1f} mm"
        )
        print("=" * 50)

        # 회차별 비교를 위해 결과를 파일로 남긴다.
        import datetime

        stamp = datetime.datetime.now().strftime("%m%d_%H%M")
        report_path = f"verify_result_{stamp}.txt"

        with open(report_path, "w", encoding="utf-8") as report:
            report.write(f"측정 개수: {len(samples)}\n")

            report.write(
                f"평균 위치: {mean[0]:+.4f}, "
                f"{mean[1]:+.4f}, {mean[2]:+.4f} m\n"
            )

            report.write(
                f"축별 산포: X={spread[0] * 1000:.1f} "
                f"Y={spread[1] * 1000:.1f} "
                f"Z={spread[2] * 1000:.1f} mm\n"
            )

            report.write(f"평균 오차: {errors.mean() * 1000:.1f} mm\n")
            report.write(f"최대 오차: {errors.max() * 1000:.1f} mm\n\n")

            report.write("측정값:\n")

            for index, position in enumerate(samples, start=1):
                report.write(
                    f"  {index:2d}  "
                    f"{position[0]:+.4f} "
                    f"{position[1]:+.4f} "
                    f"{position[2]:+.4f}\n"
                )

        print(f"\n결과를 저장했습니다: {report_path}")

    else:
        print("\n측정이 2회 미만이라 통계를 낼 수 없습니다.")

    # 종료 순서가 중요하다.
    # executor를 멈추고 spin 스레드가 실제로 끝날 때까지 기다린 뒤에
    # 노드를 파괴해야 한다. 기다리지 않으면 종료 시
    # "terminate called without an active exception / Aborted (core dumped)"
    # 가 발생한다 (결과 출력에는 영향이 없지만 오류로 오해하기 쉽다).
    executor.shutdown()
    spin_thread.join(timeout=2.0)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
