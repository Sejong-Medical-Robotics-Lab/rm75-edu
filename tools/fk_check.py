#!/usr/bin/env python3
"""오프라인 기구학 검증기 — ROS 없이 rm_75 URDF 로 FK/수치 IK 를 돌려
저장소의 상수(HOME 자세·DOWN 쿼터니언·예제/미션 좌표)의 도달성을 검증한다.

MoveIt(KDL)과 동일한 기구학 체인을 URDF 원점·축에서 직접 구성하므로,
여기서 IK 가 수렴하는 목표는 demo 모드에서도 Plan 가능성이 높다
(충돌 제외 — 충돌은 Scene 몫).

사용:  python3 tools/fk_check.py
"""
import math
import os
import re
import xml.etree.ElementTree as ET

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
XACRO = os.path.join(ROOT, "rm_description", "urdf", "rm_75.urdf.xacro")

# ── 저장소 상수(이 파일이 검증의 단일 기준) ─────────────────────────
HOME = [0.0, 0.8395, 0.0, 1.4442, 0.0, 0.8513, 0.0]   # SRDF 'home' (제조사 forward)
ZERO = [0.0] * 7
# 그리퍼 TCP: Link7 원점에서 공구 방향(+z)으로의 오프셋 [m]
TCP_OFFSET = 0.11
# '아래 보기' 자세 후보: 공구 +z 가 월드 -z, 공구 +x 는 월드 +x
DOWN_Q = (1.0, 0.0, 0.0, 0.0)          # (x, y, z, w)
# 예제·미션 좌표 (TCP 기준, base_link 좌표계) — 교재 5.5/7.2 감각과 정합
TARGETS = {
    "예제 목표 P":  (0.32,  0.00, 0.25),
    "A 접근":       (0.34,  0.16, 0.15),
    "A 파지":       (0.34,  0.16, 0.02),
    "B 접근":       (0.34, -0.16, 0.15),
    "B 내려놓기":   (0.34, -0.16, 0.03),
}


def load_chain():
    src = open(XACRO).read()
    src = src.replace("$(arg link7_type)", "Link7")
    # 조건 블록 평가: Link7(기본형)만 유지, 6f/6fb 변형 블록 제거
    src = re.sub(r"<xacro:if value=\"\$\{ link7_type =='Link7_6fb?' \}\">.*?</xacro:if>",
                 "", src, flags=re.S)
    src = re.sub(r"</?xacro:[^>]*>", "", src)
    r = ET.fromstring(src)
    joints = {}
    for j in r.findall("joint"):
        if j.get("type") != "revolute":
            continue
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
        ax = [float(v) for v in j.find("axis").get("xyz").split()]
        lim = j.find("limit")
        joints[j.get("name")] = dict(
            xyz=xyz, rpy=rpy, axis=np.array(ax, float),
            lo=float(lim.get("lower")), hi=float(lim.get("upper")))
    chain = [joints[f"joint{i}"] for i in range(1, 8)]
    return chain


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def axis_rot(axis, th):
    x, y, z = axis / np.linalg.norm(axis)
    c, s, C = math.cos(th), math.sin(th), 1 - math.cos(th)
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def fk(chain, q, tcp=TCP_OFFSET):
    T = np.eye(4)
    for jd, th in zip(chain, q):
        A = np.eye(4)
        A[:3, :3] = rpy_mat(*jd["rpy"])
        A[:3, 3] = jd["xyz"]
        R = np.eye(4)
        R[:3, :3] = axis_rot(jd["axis"], th)
        T = T @ A @ R
    tool = np.eye(4)
    tool[2, 3] = tcp
    return T @ tool


def quat_to_mat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def ik(chain, target_p, target_R, q0, iters=300):
    q = np.array(q0, float)
    lo = np.array([j["lo"] for j in chain])
    hi = np.array([j["hi"] for j in chain])
    for _ in range(iters):
        T = fk(chain, q)
        p, R = T[:3, 3], T[:3, :3]
        ep = np.asarray(target_p) - p
        Re = target_R @ R.T
        eo = 0.5 * np.array([Re[2, 1] - Re[1, 2],
                             Re[0, 2] - Re[2, 0],
                             Re[1, 0] - Re[0, 1]])
        err = np.concatenate([ep, eo])
        if np.linalg.norm(ep) < 1e-4 and np.linalg.norm(eo) < 1e-3:
            return q, True
        # 수치 야코비안
        J = np.zeros((6, 7))
        d = 1e-5
        for k in range(7):
            q2 = q.copy()
            q2[k] += d
            T2 = fk(chain, q2)
            dp = (T2[:3, 3] - p) / d
            dR = T2[:3, :3] @ R.T
            do = 0.5 * np.array([dR[2, 1] - dR[1, 2],
                                 dR[0, 2] - dR[2, 0],
                                 dR[1, 0] - dR[0, 1]]) / d
            J[:, k] = np.concatenate([dp, do])
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), err)
        q = np.clip(q + np.clip(dq, -0.2, 0.2), lo, hi)
    return q, False


def main():
    chain = load_chain()
    print(f"체인 로드: revolute {len(chain)}개")
    Tz = fk(chain, ZERO)
    Th = fk(chain, HOME)
    print(f"FK zero: TCP=({Tz[0,3]:+.3f},{Tz[1,3]:+.3f},{Tz[2,3]:+.3f})"
          f"  공구 z축(월드)={np.round(Tz[:3,2],3)}")
    print(f"FK home: TCP=({Th[0,3]:+.3f},{Th[1,3]:+.3f},{Th[2,3]:+.3f})"
          f"  공구 z축(월드)={np.round(Th[:3,2],3)}")
    Rd = quat_to_mat(DOWN_Q)
    print(f"DOWN_Q 공구 z축 목표: {np.round(Rd[:3,2],3)} (기대: [0,0,-1])")
    ok_all = True
    for name, p in TARGETS.items():
        q, ok = ik(chain, p, Rd, HOME)
        T = fk(chain, q)
        e = np.linalg.norm(T[:3, 3] - np.array(p)) * 1000
        print(f"  {name:10s} {p} → IK {'수렴' if ok else '실패'}"
              f" (잔차 {e:.1f} mm, q1..4={np.round(q[:4],2)})")
        ok_all &= ok
    print("전체 도달성:", "✓ 통과" if ok_all else "✗ 조정 필요")


if __name__ == "__main__":
    main()
