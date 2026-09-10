"""계란판 형상의 단일 출처 (T4-3, 2차). 생성기와 검증기가 공용.

원칙 (2026-09-11 사용자 결정)
  계란판은 **강체 실물**이다 — 수평, 직사각, 컵 15개 동일. 티칭 격자의 왜곡(사이각 84.26°, z 기울기)은
  로봇 티칭 오차이지 컨테이너의 성질이 아니다. 컨테이너를 그 오차에 맞추면 오차가 숨는다.
  컨테이너는 규칙적으로 만들고, 과실은 놓이는 자리에 그대로 둔다 — 과실이 컵에서 벗어나는 만큼이
  티칭 오차의 눈에 보이는 크기다.

좌표계
  애셋 프레임 = 계란판 자체 프레임. 원점 = slot 0 컵 중심의 바로 아래 테이블 상판(z=0).
  열(slot%3) 은 -x, 행(slot//3) 은 -y 방향 (티칭 격자와 같은 방향 관례). 컵 15개는 축 정렬 직교 격자.
  씬 배치(layout_layer.usd) = 아래 fit 의 (translate, z축 회전).

피치
  열 |slot1-slot0| = 59.8mm, 행 |slot3-slot0| = 51.2mm — 플래너 상수의 **크기**만 쓴다. 각도는 쓰지 않는다.
  (컨테이너 실측 치수는 기록이 없다. 3점 티칭 오차는 각도에 주로 들어가고 거리에는 덜 들어간다고 본다.)

자세 fit
  티칭 ee 격자 15점 + 평균 ee→과실 변위(런 7 실측 6개) 를 목표로, 피치 고정 직교 격자를 강체(회전+평행이동)
  최소자승(Kabsch)으로 맞춘다. 과실 6개로 직접 맞추지 않는 이유: 분면별 매달린 깊이 차(±4mm x)가
  행 그룹별로 달라 가짜 전단이 섞인다. 격자 왜곡은 fit 이 양쪽 축으로 나눠 갖는다.
"""
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "src", "strawberry_motion", "scripts"))
from harvest_motion_params import (  # noqa: E402
    TAUGHT_SLOT0_PLACE_REFERENCE_POSX_MM_DEG as _S0,
    TAUGHT_SLOT1_PLACE_REFERENCE_POSX_MM_DEG as _S1,
    TAUGHT_SLOT3_PLACE_REFERENCE_POSX_MM_DEG as _S3,
    TAUGHT_TRAY_SLOT_COUNT,
)

ROWS, COLS = 5, 3
assert ROWS * COLS == TAUGHT_TRAY_SLOT_COUNT

_EE0 = np.array(_S0[:3]) / 1000.0
V_COL_TAUGHT = (np.array(_S1[:3]) - np.array(_S0[:3])) / 1000.0   # 티칭 열 축 (왜곡 포함)
H_ROW_TAUGHT = (np.array(_S3[:3]) - np.array(_S0[:3])) / 1000.0   # 티칭 행 축 (왜곡 포함)
PITCH_COL_M = float(np.linalg.norm(V_COL_TAUGHT[:2]))               # 0.0598
PITCH_ROW_M = float(np.linalg.norm(H_ROW_TAUGHT[:2]))               # 0.0512

# 1차 출처: 런 7 (20260910T140636-71389d3a) Kit 로그 `[bridge] RELEASE ... frozen at` (m)
RUN_ID = "20260910T140636-71389d3a"
FRUIT_REST_M = {
    0:  np.array([0.7683, 0.0883, 0.0365]),
    1:  np.array([0.7066, 0.0946, 0.0372]),
    6:  np.array([0.7505, -0.0087, 0.0310]),
    7:  np.array([0.7012, -0.0052, 0.0308]),
    12: np.array([0.7425, -0.1095, 0.0242]),
    13: np.array([0.6821, -0.1066, 0.0256]),
}

# 과실 정지 자세: 파지 자세에서 수직축 기준 ~82° 회전(장축은 수직 유지). 중심 기준 밑끝 -33.1mm, 줄기끝 +29.5mm.
FRUIT_BOTTOM_BELOW_CENTER_M = 0.0331
FRUIT_XY_RATIO = 0.82                     # 좁은 방향/넓은 방향 (22.2/27.0). 회전 후 넓은 쪽이 world y 근방
_PROFILE_DZ_M = np.array([-0.0331, -0.0315, -0.0284, -0.0252, -0.0221, -0.0190, -0.0159, -0.0127, -0.0096,
                          -0.0065, -0.0033, -0.0002, 0.0030, 0.0060, 0.0092, 0.0123, 0.0155, 0.0186, 0.0217])
_PROFILE_R_M = np.array([0.0000, 0.00985, 0.01295, 0.0145, 0.0158, 0.0166, 0.01805, 0.0194, 0.0212,
                         0.02295, 0.0247, 0.02645, 0.0269, 0.0270, 0.0268, 0.0258, 0.0239, 0.0213, 0.0179])


def fruit_max_radius_m(dz):
    return float(np.interp(dz, _PROFILE_DZ_M, _PROFILE_R_M, left=0.0, right=0.0))


# 컵 (15개 동일, 애셋 프레임 z = 테이블 상판 기준)
PLATE_TOP_Z_M = 0.016        # 판 윗면 = 림 높이. 가장 낮게 놓이는 과실(중심 z 24mm)의 그 높이 반경 21.9 < ry 23
CUP_FLOOR_Z_M = 0.001        # 컵 바닥 (상판 위 — 빈 컵에 테이블이 비치지 않게)
SKIRT_BOTTOM_Z_M = -0.003    # 스커트 아랫변 (상판 3mm 아래로 앉힘)
CUP_RINGS_M = [              # (z, rx, ry): 림 → 바닥. rx 는 열(-x) 방향, ry 는 행(-y) 방향
    (PLATE_TOP_Z_M, 0.026, 0.023),   # 피치 59.8 / 51.2 안에서 능선 7.8 / 5.2mm 가 남는 최대
    (0.0110, 0.021, 0.019),
    (0.0060, 0.017, 0.015),
    (CUP_FLOOR_Z_M, 0.013, 0.012),
]
CUP_SIDES = 16
PLATE_MARGIN_M = 0.006

# 자세 각도 강제. None = 티칭 격자에 최소자승 fit (-7.24°, 컵 중심 잔차 최대 5.3mm).
#   0.0   = 로봇 베이스 축에 정렬 — 잔차 최대 16.7mm (행 0·4 과실이 x 로 ±16mm, 능선 위에 걸린다)
#   -3.30 = 티칭 열 축에 정렬 — 잔차 최대 10.3mm
YAW_OVERRIDE_DEG = None


def cup_center_asset_m(slot):
    """애셋 프레임 컵 중심 (xy, z=0). 열은 -x, 행은 -y."""
    r, c = divmod(slot, COLS)
    return np.array([-c * PITCH_COL_M, -r * PITCH_ROW_M, 0.0])


def _taught_targets_m():
    """티칭 ee 격자 15점 + 평균 ee→과실 변위 (xy). fit 의 목표."""
    ee = {s: _EE0 + divmod(s, COLS)[0] * H_ROW_TAUGHT + divmod(s, COLS)[1] * V_COL_TAUGHT for s in range(15)}
    mean_off = np.mean([FRUIT_REST_M[s] - ee[s] for s in FRUIT_REST_M], axis=0)
    return {s: (ee[s] + mean_off)[:2] for s in range(15)}, mean_off


def fit_pose():
    """강체 2D Kabsch: 애셋 격자 → world. 반환 (translate_m[3], yaw_rad, residual_mm{slot})."""
    tgt, _ = _taught_targets_m()
    L = np.array([cup_center_asset_m(s)[:2] for s in range(15)])
    P = np.array([tgt[s] for s in range(15)])
    Lc, Pc = L - L.mean(0), P - P.mean(0)
    if YAW_OVERRIDE_DEG is None:
        U, _, Vt = np.linalg.svd(Lc.T @ Pc)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        Rm = Vt.T @ np.diag([1.0, d]) @ U.T
    else:
        cy, sy = np.cos(np.radians(YAW_OVERRIDE_DEG)), np.sin(np.radians(YAW_OVERRIDE_DEG))
        Rm = np.array([[cy, -sy], [sy, cy]])
    t = P.mean(0) - Rm @ L.mean(0)
    yaw = float(np.arctan2(Rm[1, 0], Rm[0, 0]))
    res = {s: (P[s] - (Rm @ L[s] + t)) * 1000.0 for s in range(15)}
    return np.array([t[0], t[1], 0.0]), yaw, res


def world_to_asset(p_world, translate, yaw):
    c, s = np.cos(-yaw), np.sin(-yaw)
    q = np.asarray(p_world) - translate
    return np.array([c * q[0] - s * q[1], s * q[0] + c * q[1], q[2]])


def cup_radii_at_z(z):
    """애셋 프레임 높이 z 에서의 컵 (rx, ry). 바닥 아래는 None, 림 위는 림 값."""
    if z >= CUP_RINGS_M[0][0]:
        return CUP_RINGS_M[0][1], CUP_RINGS_M[0][2]
    for (z1, x1, y1), (z2, x2, y2) in zip(CUP_RINGS_M, CUP_RINGS_M[1:]):
        if z2 <= z <= z1:
            t = (z1 - z) / (z1 - z2)
            return x1 + t * (x2 - x1), y1 + t * (y2 - y1)
    return None
