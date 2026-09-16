#!/usr/bin/env python3
"""손목 D455 컬러 카메라 투영 대조 — 시뮬 화면 = 실기 화면인지 수치로 본다 (2026-09-16).

런타임 노드가 아니다. **Isaac Sim·GPU·cuRobo·ROS 없이** 돈다 (numpy + pxr + pyyaml).

무엇을 하나
-----------
스캔 자세에서 손목 카메라가 보드의 어느 점을 화면 어디에 담는지 계산해, 실기 비전 노드
화면(사진)에서 읽은 값과 **화면 비율**로 대조한다. 카메라를 옮기지 않는다 — 지금 배치가
실기와 같은 방향인지 확인하는 용도다.

출처 (전부 읽기만 한다)
----------------------
  * 씬 USD        : 로봇 배치, 카메라 체인 변환, focalLength·aperture, 보드 위치·크기
  * URDF          : 팔·그리퍼 FK
  * 스캔 자세 YAML: 분면 4자세 (endpoint_joints_deg)
  * 구 YAML       : 핑거 끝 = custom_part 콜리전 구 중 |y| 최대인 것

자체 검산
---------
관절 0 에서 계산한 FK 와 USD 에 기록된 링크 변환을 비교한다. URDF 임포트 당시 자세가
관절 0 이므로 둘은 같아야 한다 — 어긋나면 FK 나 씬 배치가 틀어진 것이니 표를 믿지 마라.

실행 (리포 루트에서)
-------------------
  python3 src/strawberry_motion/scripts/check_wrist_camera_projection.py
  python3 src/strawberry_motion/scripts/check_wrist_camera_projection.py --pose root/nw
  python3 src/strawberry_motion/scripts/check_wrist_camera_projection.py --joints-deg 88,-95,130,176,-31,93
  python3 src/strawberry_motion/scripts/check_wrist_camera_projection.py --ref-cross 0.49,0.50 --json out.json

한계
----
렌즈 왜곡·주점 오프셋은 모델에 없다 (USD 카메라는 핀홀, 09-16 기준 apertureOffset 0).
기준값(--ref-*)은 사람이 사진에서 읽는 값이다 — 기본값은 눈대중이라 정밀하지 않다.
"""
import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

DEF_SCENE = os.path.join(REPO, "strawberry_harvest/scenes/main_scene.usd")
DEF_URDF = os.path.join(REPO, "src/strawberry_motion/config/curobo/e0509_gripper.urdf")
DEF_SPHERES = os.path.join(REPO, "src/strawberry_motion/config/curobo/e0509_spheres.yml")
DEF_YAML = os.path.join(REPO, "src/strawberry_motion/config/scan_pose_candidates_refit_candidate.yaml")

EE_PRIM = "/World/robot_assembly/rh_p12_rn_base"
CAM_PRIM = EE_PRIM + "/rsd455/RSD455/Camera_OmniVision_OV9782_Color"
DEPTH_PRIM = EE_PRIM + "/rsd455/RSD455/Camera_Pseudo_Depth"
MODULE_PRIM = EE_PRIM + "/rsd455"
BOARD_PRIM = "/World/lab_environment/whiteboard"
ARM_JOINT_PRIMS = ["/World/robot_assembly/joints/joint_%d" % i for i in range(1, 7)]

EE_LINK = "gripper_rh_p12_rn_base"
FINGER_LINKS = ("gripper_left_custom_part", "gripper_right_custom_part")
GRIPPER_DRIVE_JOINT = "gripper_rh_r1"
#: gripper_joint_publisher.py 의 변환 — stroke 700 = 1.0 rad (0 = 열림)
STROKE_TO_RAD = 1.0 / 700.0
#: scan_executor_node._GRIPPER_APPROACH_POS — 스캔 이동 중 개도
DEF_STROKE = 600
#: run_nodes.sh 의 플래너 파라미터 ee_to_tcp_offset_m
DEF_TCP_OFFSET_M = 0.236

#: 실기 비전 노드 화면(세 번째 사진)에서 **눈대중으로** 읽은 화면 비율 (가로, 세로).
#: 정밀 측정값이 아니다 — 원본 프레임에서 픽셀로 다시 재면 --ref-* 로 넘겨라.
REF_CROSS = (0.49, 0.50)
REF_FINGERS = (0.46, 0.49)
REF_NOTE = "눈대중 ±0.02 — 원본 프레임에서 재측정 필요"

#: 핑거 끝 두 점의 중점. 표와 대조 기준이 같은 이름을 봐야 해서 상수로 둔다.
FINGER_MID_LABEL = "핑거 끝 중점"


# ---------------------------------------------------------------- URDF FK
def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_rot(axis, th):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(th) * k + (1.0 - math.cos(th)) * k @ k


def _tf(rot, trans):
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = trans
    return m


class UrdfFk:
    """URDF 직렬 체인 FK. revolute·fixed·mimic 만 다룬다 (이 로봇에 그것뿐이다)."""

    def __init__(self, urdf_path):
        root = ET.parse(urdf_path).getroot()
        self.joints = {}
        for j in root.findall("joint"):
            org = j.find("origin")
            ax = j.find("axis")
            mim = j.find("mimic")
            self.joints[j.get("name")] = {
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "xyz": [float(v) for v in (org.get("xyz", "0 0 0").split()
                                           if org is not None else ["0", "0", "0"])],
                "rpy": [float(v) for v in (org.get("rpy", "0 0 0").split()
                                           if org is not None else ["0", "0", "0"])],
                "axis": [float(v) for v in ax.get("xyz").split()] if ax is not None else [0, 0, 1],
                "mimic": (mim.get("joint"), float(mim.get("multiplier", 1.0)),
                          float(mim.get("offset", 0.0))) if mim is not None else None,
            }

    def _chain(self, link):
        names, cur = [], link
        while True:
            j = next((n for n, d in self.joints.items() if d["child"] == cur), None)
            if j is None:
                return list(reversed(names))
            names.append(j)
            cur = self.joints[j]["parent"]

    def pose(self, link, q):
        """링크의 base_link 기준 4x4 변환. q: {관절명: rad}"""
        m = np.eye(4)
        for name in self._chain(link):
            d = self.joints[name]
            m = m @ _tf(_rpy(*d["rpy"]), d["xyz"])
            if d["type"] == "revolute":
                th = q.get(name, 0.0)
                if d["mimic"]:
                    src, mul, off = d["mimic"]
                    th = q.get(src, 0.0) * mul + off
                m = m @ _tf(_axis_rot(d["axis"], th), [0, 0, 0])
        return m


def arm_q(joints_deg, stroke):
    q = {"joint_%d" % (i + 1): math.radians(v) for i, v in enumerate(joints_deg)}
    q[GRIPPER_DRIVE_JOINT] = float(stroke) * STROKE_TO_RAD
    return q


# ---------------------------------------------------------------- USD 읽기
def read_scene(scene_path):
    try:
        from pxr import Usd, UsdGeom
    except ImportError:
        sys.exit("pxr(USD python)를 못 찾았다. Isaac Sim 의 python 환경이나 usd-core 가 필요하다:\n"
                 "  pip install usd-core")
    stage = Usd.Stage.Open(scene_path)
    if stage is None:
        sys.exit("씬을 열지 못했다: %s" % scene_path)
    tc = Usd.TimeCode.Default()

    def world(path):
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            sys.exit("프림이 없다: %s (씬이 바뀌었는지 확인)" % path)
        # pxr 은 행벡터 규약 — numpy(열벡터)로 쓰려면 전치한다
        return np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(tc), dtype=float).T

    ee_w = world(EE_PRIM)
    ee_inv = np.linalg.inv(ee_w)
    out = {
        "authored_ee": ee_w,
        "cam_in_ee": ee_inv @ world(CAM_PRIM),
        "depth_in_ee": ee_inv @ world(DEPTH_PRIM),
        "module_in_ee": ee_inv @ world(MODULE_PRIM),
    }

    cam_prim = stage.GetPrimAtPath(CAM_PRIM)
    intr = {}
    for attr in ("focalLength", "horizontalAperture", "verticalAperture",
                 "horizontalApertureOffset", "verticalApertureOffset"):
        a = cam_prim.GetAttribute(attr)
        intr[attr] = float(a.Get()) if a and a.Get() is not None else 0.0
    out["intrinsics"] = intr

    bbox = UsdGeom.BBoxCache(tc, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = bbox.ComputeWorldBound(stage.GetPrimAtPath(BOARD_PRIM)).ComputeAlignedRange()
    lo, hi = rng.GetMin(), rng.GetMax()
    out["board"] = {"min": [lo[0], lo[1], lo[2]], "max": [hi[0], hi[1], hi[2]]}

    rest = []
    for path in ARM_JOINT_PRIMS:
        prim = stage.GetPrimAtPath(path)
        a = prim.GetAttribute("drive:angular:physics:targetPosition") if prim and prim.IsValid() else None
        rest.append(float(a.Get()) if a and a.Get() is not None else 0.0)
    out["rest_deg"] = rest          # USD physics 각도 드라이브는 도 단위
    return out


def finger_tip_in_link(spheres_path):
    """custom_part 링크 프레임에서 핑거 끝 = |y| 가 가장 큰 콜리전 구의 중심."""
    import yaml
    with open(spheres_path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    coll = data.get("collision_spheres", data)
    tips = {}
    for link in FINGER_LINKS:
        if link not in coll:
            sys.exit("구 정의에 %s 가 없다: %s" % (link, spheres_path))
        best = max(coll[link], key=lambda s: abs(float(s["center"][1])))
        tips[link] = (np.array([float(v) for v in best["center"]]), float(best["radius"]))
    return tips


def load_scan_poses(yaml_path):
    import yaml
    with open(yaml_path, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)["scan_pose_candidates"]
    return {t["cell_id"]: [float(v) for v in t["endpoint_joints_deg"]]
            for t in cfg.get("targets", []) if t.get("endpoint_joints_deg")}


# ---------------------------------------------------------------- 투영
class Pinhole:
    """USD 카메라 투영. USD 카메라는 -Z 를 보고 +Y 가 위, +X 가 오른쪽이다."""

    def __init__(self, cam_world, intr, res):
        rot = cam_world[:3, :3].copy()
        for c in range(3):                      # 스케일 제거 (씬에 1.0 아닌 축이 있다)
            rot[:, c] /= np.linalg.norm(rot[:, c])
        self.rot, self.pos = rot, cam_world[:3, 3].copy()
        self.f = intr["focalLength"]
        self.ha, self.va = intr["horizontalAperture"], intr["verticalAperture"]
        self.hoff, self.voff = intr["horizontalApertureOffset"], intr["verticalApertureOffset"]
        self.w, self.h = res

    @property
    def fov_deg(self):
        return (math.degrees(2 * math.atan(self.ha / (2 * self.f))),
                math.degrees(2 * math.atan(self.va / (2 * self.f))))

    @property
    def forward(self):
        return -self.rot[:, 2]                  # 광축

    def project(self, point_world):
        p = self.rot.T @ (np.asarray(point_world, float) - self.pos)
        depth = -p[2]                           # 카메라 앞 = -Z
        if depth <= 1e-6:
            return None, depth
        u_mm = self.f * p[0] / depth + self.hoff
        v_mm = self.f * p[1] / depth + self.voff
        fx = 0.5 + u_mm / self.ha               # 0..1, 왼쪽에서
        fy = 0.5 - v_mm / self.va               # 0..1, 위에서
        return (fx, fy), depth


def landmarks(board, ee_w, fk, q, tips, tcp_offset_m):
    """(이름, 월드 좌표, 분류) 목록."""
    x0, _y0, z0 = board["min"]
    x1, _, z1 = board["max"]
    yb = board["max"][1]                        # 보드 앞면 (로봇 쪽)
    xm, zm = 0.5 * (x0 + x1), 0.5 * (z0 + z1)
    items = [
        ("보드 십자 교차점", [xm, yb, zm], "board"),
        ("보드 좌상(NW 바깥)", [x0, yb, z1], "board"),
        ("보드 우상(NE 바깥)", [x1, yb, z1], "board"),
        ("보드 좌하(SW 바깥)", [x0, yb, z0], "board"),
        ("보드 우하(SE 바깥)", [x1, yb, z0], "board"),
        ("십자 세로선 위끝", [xm, yb, z1], "board"),
        ("십자 세로선 아래끝", [xm, yb, z0], "board"),
        ("십자 가로선 왼끝", [x0, yb, zm], "board"),
        ("십자 가로선 오른끝", [x1, yb, zm], "board"),
    ]
    tcp = ee_w @ np.array([0.0, 0.0, float(tcp_offset_m), 1.0])
    items.append(("TCP (툴축 %.0fmm)" % (tcp_offset_m * 1000), tcp[:3].tolist(), "tool"))
    tip_pts = []
    for link, label in zip(FINGER_LINKS, ("왼쪽 핑거 끝", "오른쪽 핑거 끝")):
        lp = fk.pose(link, q)
        c, _r = tips[link]
        pt = (lp @ np.append(c, 1.0))[:3]
        tip_pts.append(pt)
        items.append((label, pt.tolist(), "tool"))
    # 사진에서 눈으로 읽는 "핑거가 모이는 점" 에 가장 가까운 양. TCP 는 툴축 위의 한 점이라
    # 핑거 실루엣이 만나는 곳과 다르다 — 대조 기준은 이 중점을 쓴다.
    items.append((FINGER_MID_LABEL, ((tip_pts[0] + tip_pts[1]) / 2.0).tolist(), "tool"))
    return items


# ---------------------------------------------------------------- 출력
def jsonable(obj):
    """numpy 스칼라·배열을 파이썬 기본형으로. json.dump 가 numpy.bool_ 에서 죽으면
    파일이 반만 쓰인 채 남는다 — 그 사고를 막는다."""
    if isinstance(obj, dict):
        return {str(k): jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return jsonable(obj.tolist())
    if isinstance(obj, np.generic):
        return obj.item()
    return obj



def fmt_frac(fr, res):
    if fr is None:
        return "카메라 뒤", ""
    fx, fy = fr
    inside = 0.0 <= fx <= 1.0 and 0.0 <= fy <= 1.0
    return ("%5.1f%% / %5.1f%%" % (fx * 100, fy * 100),
            "%4d,%4d%s" % (round(fx * res[0]), round(fy * res[1]), "" if inside else "  (밖)"))


def tool_summary(scene, fk, tips, args, res):
    """카메라와 그리퍼는 같은 링크에 붙어 있다 — 화면 속 핑거 위치는 팔 자세와 무관하다.

    ee 프레임을 그대로 '월드' 로 써서 한 번만 계산한다. 자세별 표의 공구 행과 같은 값이
    나와야 하고, 실제로 같다 (그것이 이 함수의 요지다).
    """
    print("\n" + "=" * 78)
    print("카메라 ↔ 그리퍼 고정 관계 — 팔 자세와 무관한 값")
    print("=" * 78)
    cam = Pinhole(scene["cam_in_ee"], scene["intrinsics"], res)
    print("%-16s %14s %12s %10s" % ("항목", "가로/세로 비율", "픽셀", "거리(mm)"))
    print("-" * 56)
    out = {}
    for stroke in sorted({args.gripper_stroke, 0, 700}):
        q = arm_q([0.0] * 6, stroke)
        # fk.pose 는 base_link 기준이다. 카메라 변환이 ee 기준이므로 ee 로 옮긴다
        # (그리퍼 관절은 ee 링크보다 아래라 ee 자세는 개도와 무관하다).
        ee_inv = np.linalg.inv(fk.pose(EE_LINK, q))
        pts = []
        for link in FINGER_LINKS:
            c, _r = tips[link]
            pts.append((ee_inv @ fk.pose(link, q) @ np.append(c, 1.0))[:3])
        mid = (pts[0] + pts[1]) / 2.0
        fr, depth = cam.project(mid)
        frac_s, px_s = fmt_frac(fr, res)
        mark = " *" if stroke == args.gripper_stroke else "  "
        print("%-16s %14s %12s %10.0f" % ("핑거 끝 중점 s%d%s" % (stroke, mark),
                                          frac_s, px_s, depth * 1000))
        out["stroke_%d" % stroke] = {"frac": list(fr) if fr else None, "depth_mm": depth * 1000}
    tcp = np.array([0.0, 0.0, float(args.tcp_offset_m)])
    fr, depth = cam.project(tcp)
    frac_s, px_s = fmt_frac(fr, res)
    print("%-16s %14s %12s %10.0f" % ("TCP", frac_s, px_s, depth * 1000))
    out["tcp"] = {"frac": list(fr) if fr else None, "depth_mm": depth * 1000}
    print("  * = --gripper-stroke 로 준 개도.  화면 중앙은 50.0% / 50.0% 다.")
    return out


def run_pose(name, joints_deg, scene, fk, tips, args, res):
    q = arm_q(joints_deg, args.gripper_stroke)
    ee_w = fk.pose(EE_LINK, q)
    cam_w = ee_w @ scene["cam_in_ee"]
    cam = Pinhole(cam_w, scene["intrinsics"], res)
    board = scene["board"]
    yb = board["max"][1]

    print("\n" + "=" * 78)
    print("자세 %s  관절 %s" % (name, ", ".join("%.2f" % v for v in joints_deg)))
    print("=" * 78)
    hf, vf = cam.fov_deg
    print("카메라 원점(월드) : %s m" % np.array2string(cam.pos, precision=4, suppress_small=True))
    print("광축              : %s   (보드 법선 +y 와 각도 %.1f°)"
          % (np.array2string(cam.forward, precision=3, suppress_small=True),
             math.degrees(math.acos(max(-1.0, min(1.0, float(cam.forward[1])))))))
    print("화각              : %.2f x %.2f deg   해상도 %dx%d" % (hf, vf, res[0], res[1]))

    depth_w = ee_w @ scene["depth_in_ee"]
    mod_w = ee_w @ scene["module_in_ee"]
    print("보드 앞면까지 y거리: 컬러 %.1f mm · 깊이원점 %.1f mm · 모듈원점 %.1f mm"
          % ((yb - cam_w[1, 3]) * 1000, (yb - depth_w[1, 3]) * 1000, (yb - mod_w[1, 3]) * 1000))

    print("\n%-22s %14s %12s %10s" % ("랜드마크", "가로/세로 비율", "픽셀", "거리(mm)"))
    print("-" * 66)
    rows = []
    for label, pt, kind in landmarks(board, ee_w, fk, q, tips, args.tcp_offset_m):
        fr, depth = cam.project(pt)
        frac_s, px_s = fmt_frac(fr, res)
        print("%-22s %14s %12s %10.0f" % (label, frac_s, px_s, depth * 1000))
        rows.append({"label": label, "kind": kind, "world_m": list(pt),
                     "frac": list(fr) if fr else None, "depth_mm": depth * 1000})

    refs = [("보드 십자 교차점", args.ref_cross, "십자 교차점"),
            (FINGER_MID_LABEL, args.ref_fingers, "핑거 끝 중점")]
    print("\n실기 사진 기준값 대조  (%s)" % REF_NOTE)
    print("%-14s %14s %14s %16s" % ("항목", "시뮬", "실기(기준)", "차이"))
    print("-" * 62)
    for key, ref, short in refs:
        row = next((r for r in rows if r["label"] == key), None)
        if row is None or row["frac"] is None or ref is None:
            print("%-14s %14s %14s %16s" % (short, "-", "-", "계산 불가"))
            continue
        fx, fy = row["frac"]
        dx, dy = (fx - ref[0]) * 100, (fy - ref[1]) * 100
        print("%-14s %6.1f%% /%6.1f%% %6.1f%% /%6.1f%%   %+5.1f / %+5.1f %%p  (%+d,%+d px)"
              % (short, fx * 100, fy * 100, ref[0] * 100, ref[1] * 100, dx, dy,
                 round(dx / 100 * res[0]), round(dy / 100 * res[1])))
        row["ref_frac"] = list(ref)
        row["delta_pp"] = [dx, dy]
    return {"pose": name, "joints_deg": list(joints_deg),
            "camera_pos_m": cam.pos.tolist(), "fov_deg": [hf, vf],
            "board_distance_mm": {
                "color": (yb - cam_w[1, 3]) * 1000,
                "pseudo_depth": (yb - depth_w[1, 3]) * 1000,
                "module_origin": (yb - mod_w[1, 3]) * 1000},
            "landmarks": rows}


def self_check(scene, fk, args):
    print("=" * 78)
    print("자체 검산 — 관절 0 에서 FK 와 USD 기록 링크 변환이 같아야 한다")
    print("=" * 78)
    fk0 = fk.pose(EE_LINK, arm_q([0.0] * 6, 0))
    usd0 = scene["authored_ee"]
    dt = np.linalg.norm(fk0[:3, 3] - usd0[:3, 3]) * 1000
    dr = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(fk0[:3, :3].T @ usd0[:3, :3]) - 1) / 2))))
    ok = bool(dt <= args.fk_tol_mm and dr <= args.fk_tol_deg)
    print("  ee 위치 차 %.4f mm · 자세 차 %.4f deg → %s" % (dt, dr, "OK" if ok else "불일치"))
    if not ok:
        print("  FK 나 씬 배치가 어긋났다. 아래 표를 믿지 마라.")

    # 투영식 검산 — 광축 위의 점은 화면 정중앙, 화각 절반만큼 튼 점은 화면 가장자리다.
    cam = Pinhole(scene["cam_in_ee"], scene["intrinsics"], (640, 480))
    hf, vf = [math.radians(v) for v in cam.fov_deg]
    d = 0.5
    probes = [("광축", [0, 0, -d], (0.5, 0.5)),
              ("좌 가장자리", [-d * math.tan(hf / 2), 0, -d], (0.0, 0.5)),
              ("우 가장자리", [d * math.tan(hf / 2), 0, -d], (1.0, 0.5)),
              ("위 가장자리", [0, d * math.tan(vf / 2), -d], (0.5, 0.0)),
              ("아래 가장자리", [0, -d * math.tan(vf / 2), -d], (0.5, 1.0))]
    worst, worst_name = 0.0, ""
    for name, p_cam, want in probes:
        world = (scene["cam_in_ee"] @ np.append(np.asarray(p_cam, float), 1.0))[:3]
        got, _dep = cam.project(world)
        err = max(abs(got[0] - want[0]), abs(got[1] - want[1])) if got else 1.0
        if err > worst:
            worst, worst_name = err, name
    proj_ok = bool(worst <= 1e-6)
    print("  투영식 검산 최대 오차 %.2e (%s) → %s" % (worst, worst_name, "OK" if proj_ok else "불일치"))
    ok = ok and proj_ok

    mod = scene["module_in_ee"][:3, 3]
    cam = scene["cam_in_ee"][:3, 3]
    dep = scene["depth_in_ee"][:3, 3]
    print("\n  ee 프레임 기준 원점 (mm)")
    print("    rsd455 모듈       %s" % np.array2string(mod * 1000, precision=1))
    print("    컬러 카메라       %s" % np.array2string(cam * 1000, precision=1))
    print("    Pseudo depth      %s" % np.array2string(dep * 1000, precision=1))
    try:
        sys.path.insert(0, os.path.join(REPO, "src/strawberry_motion"))
        from strawberry_motion.execution.subcell_pose import CAMERA_OFFSET_IN_EE_M as code_off
        d_mod = np.linalg.norm(np.array(code_off) - mod) * 1000
        d_cam = np.linalg.norm(np.array(code_off) - cam) * 1000
        print("    subcell_pose 상수 %s" % np.array2string(np.array(code_off) * 1000, precision=1))
        print("      → 모듈 원점과 %.1f mm, 컬러 카메라 원점과 %.1f mm 차이" % (d_mod, d_cam))
        if d_cam > 1.0:
            print("      camera_board_distance_mm() 는 모듈 원점을 쓴다 — 광학 원점 기준 거리는 위 표를 봐라")
    except Exception as exc:                              # noqa: BLE001
        print("    subcell_pose 상수 대조 생략 (%s)" % exc.__class__.__name__)
    return ok


def parse_pair(text, what):
    try:
        a, b = [float(v) for v in text.split(",")]
    except Exception:                                      # noqa: BLE001
        raise argparse.ArgumentTypeError("%s 는 'x,y' 형식이다: %s" % (what, text))
    return (a, b)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", default=DEF_SCENE)
    ap.add_argument("--urdf", default=DEF_URDF)
    ap.add_argument("--spheres", default=DEF_SPHERES)
    ap.add_argument("--yaml", default=DEF_YAML)
    ap.add_argument("--pose", default="all",
                    help="rest | root/nw | root/ne | root/se | root/sw | all (기본 all)")
    ap.add_argument("--joints-deg", default=None,
                    help="쉼표 6개. 주면 --pose 대신 이 자세만 본다")
    ap.add_argument("--gripper-stroke", type=int, default=DEF_STROKE,
                    help="0=열림 700=닫힘 (기본 %d = 스캔 이동 중 개도)" % DEF_STROKE)
    ap.add_argument("--tcp-offset-m", type=float, default=DEF_TCP_OFFSET_M,
                    help="ee → TCP 툴축(+z) 거리 (기본 %.3f = run_nodes.sh)" % DEF_TCP_OFFSET_M)
    ap.add_argument("--res", default="640x480", help="카메라 해상도 (기본 640x480)")
    ap.add_argument("--ref-cross", default="%g,%g" % REF_CROSS,
                    help="실기 사진의 십자 교차점 화면 비율 'x,y'")
    ap.add_argument("--ref-fingers", default="%g,%g" % REF_FINGERS,
                    help="실기 사진의 핑거 수렴점 화면 비율 'x,y'")
    ap.add_argument("--fk-tol-mm", type=float, default=1.0)
    ap.add_argument("--fk-tol-deg", type=float, default=0.1)
    ap.add_argument("--json", default=None, help="결과를 이 경로에 JSON 으로 쓴다")
    args = ap.parse_args()

    args.ref_cross = parse_pair(args.ref_cross, "--ref-cross")
    args.ref_fingers = parse_pair(args.ref_fingers, "--ref-fingers")
    try:
        res = tuple(int(v) for v in args.res.lower().split("x"))
        assert len(res) == 2
    except Exception:                                      # noqa: BLE001
        sys.exit("--res 는 640x480 형식이다")

    scene = read_scene(args.scene)
    fk = UrdfFk(args.urdf)
    tips = finger_tip_in_link(args.spheres)

    print("씬   : %s" % os.path.relpath(args.scene, REPO))
    print("URDF : %s" % os.path.relpath(args.urdf, REPO))
    b = scene["board"]
    print("보드 : x %.3f~%.3f · 앞면 y %.3f · z %.3f~%.3f (m, USD 바운딩박스)"
          % (b["min"][0], b["max"][0], b["max"][1], b["min"][2], b["max"][2]))
    print("그리퍼 stroke %d → %.3f rad" % (args.gripper_stroke, args.gripper_stroke * STROKE_TO_RAD))
    ok = self_check(scene, fk, args)

    if args.joints_deg:
        vals = [float(v) for v in args.joints_deg.split(",")]
        if len(vals) != 6:
            sys.exit("--joints-deg 는 6개다")
        poses = [("직접 지정", vals)]
    else:
        table = load_scan_poses(args.yaml)
        table["rest"] = scene["rest_deg"]
        if args.pose == "all":
            order = ["rest", "root/nw", "root/ne", "root/se", "root/sw"]
            poses = [(k, table[k]) for k in order if k in table]
        elif args.pose in table:
            poses = [(args.pose, table[args.pose])]
        else:
            sys.exit("모르는 자세: %s (있는 것: %s)" % (args.pose, ", ".join(sorted(table))))

    tool = tool_summary(scene, fk, tips, args, res)
    out = [run_pose(name, joints, scene, fk, tips, args, res) for name, joints in poses]

    if args.json:
        payload = jsonable({"scene": args.scene, "self_check_ok": bool(ok),
                            "gripper_stroke": args.gripper_stroke,
                            "resolution": list(res), "board": scene["board"],
                            "intrinsics": scene["intrinsics"],
                            "ref": {"cross": list(args.ref_cross),
                                    "fingers": list(args.ref_fingers), "note": REF_NOTE},
                            "tool_in_camera": tool,
                            "poses": out})
        text = json.dumps(payload, ensure_ascii=False, indent=2)   # 먼저 직렬화, 그 다음 쓰기
        with open(args.json, "w", encoding="utf-8") as fh:
            fh.write(text + "\n")
        print("\nJSON 기록: %s" % args.json)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
