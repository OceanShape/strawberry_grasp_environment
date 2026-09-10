"""계란판 형상의 단일 출처 (T4-3). 생성기와 검증기가 공용.

좌표계
  로컬 원점 = slot 0 컵 중심 (아래 fit 결과), 축은 world 와 동일.
  layout_layer.usd 의 translate 가 이 원점의 world 위치다.

격자
  컵 15개 = 5행 × 3열, slot = row*3 + col.
  격자 벡터는 플래너 상수 TAUGHT_SLOT{0,1,3}_PLACE_REFERENCE_POSX_MM_DEG 에서 그대로 온다
  (열 축 v = slot1 - slot0, 행 축 h = slot3 - slot0). 새 좌표를 만들지 않는다 (§2 부수규칙 2).
  ⚠️ 이 격자는 두 축 사이각이 84.26° 라 평행사변형이다 — 실기 3점 수동 티칭 오차의 재현.
     컵도 같은 격자로 놓아야 과실과 맞는다.

원점 fit
  격자 상수는 그리퍼 밑동(ee) 위치이고, 과실은 툴 축으로 ~250mm 앞에 매달려 놓인다.
  그래서 컵 격자의 원점은 ee 상수가 아니라 **최종 런(런 7)의 과실 정지 위치 6개**로 최소자승 fit 한다:
      F0 = mean( F_s - row_s*h - col_s*v )
  과실별 매달린 깊이 차(분면별 접근 기울기, 최대 8.7mm)는 잔차로 남는다(≤6mm). 점유 컵 6개는
  그 잔차만큼 격자에서 옮겨 실측 과실 위치에 맞춘다 (cup_shift_local_m).
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

# 격자 벡터 (m)
V_COL = (np.array(_S1[:3]) - np.array(_S0[:3])) / 1000.0   # slot%3 방향  ≈ (-59.7, +3.4, +0.9)mm
H_ROW = (np.array(_S3[:3]) - np.array(_S0[:3])) / 1000.0   # slot//3 방향 ≈ (-8.0, -50.6, -2.5)mm

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

# 과실 정지 자세 (런 7 해석): 파지 자세에서 수직축 기준 ~82° 회전, 장축은 그대로 수직.
#   world 전폭 x 44.5 / y 53.8 / z 62.6 mm, 중심 기준 밑끝 -33.1mm, 줄기끝 +29.5mm, 최대 반경은 중심 +6mm 부근.
FRUIT_BOTTOM_BELOW_CENTER_M = 0.0331
FRUIT_HALF_X_M, FRUIT_HALF_Y_M = 0.0223, 0.0269

# 컵 형상 (컵 중심 = 과실 정지 중심 fit 값, 그 기준 상대 높이)
CUP_RIM_DZ_M = -0.012        # 판 윗면 = 과실 중심 12mm 아래 (과실 반경 그 높이에서 ~19mm)
CUP_BOTTOM_DZ_M = -0.036     # 컵 바닥 목표 = 과실 밑끝(-33.1) 보다 3mm 아래 …
# … 이지만 격자 z 기울기(행당 -2.5mm) 때문에 그러면 컵 바닥이 **전부 테이블 상판(z=0) 아래**로 간다
# (slot0 -0.6mm ~ slot12 -10.4mm). 상판은 불투명이라 빈 컵 안에 테이블 면이 비친다.
# 그래서 컵 바닥은 상판 위 +1mm 를 하한으로 둔다. 낮은 행의 과실은 밑끝이 컵 바닥보다 아래로
# 잠기는데(최대 ~10mm), 그 부분은 바닥판·테이블 안이라 보이지 않는다 — 과실 정지 위치(런 실측)는 건드리지 않는다.
CUP_FLOOR_MIN_WORLD_Z_M = 0.001
CUP_RIM_RX_M, CUP_RIM_RY_M = 0.026, 0.023     # 열 피치 59.8 / 행 피치 51.2 안에서 능선이 남는 최대 크기
CUP_WALL_MARGIN_M = 0.003                     # 컵 벽 = 과실 반경 프로파일 + 이 여유 (런 간 팔 산포 ~2mm 를 덮는다)
CUP_RIM_EDGE_GAP_M = 0.001                    # 림이 셀 경계에 남기는 최소 여유
CUP_BOT_MIN_RX_M, CUP_BOT_MIN_RY_M = 0.011, 0.010   # 바닥 링 최소 (과실 밑끝 아래로 내려간 컵)
CUP_RINGS = 4                                 # 림 / 1/3 / 2/3 / 바닥
FRUIT_XY_RATIO = 0.82                          # 과실 단면: 좁은 방향/넓은 방향 (22.2/27.0). 회전 후 넓은 쪽이 world y

# 과실 최대 반경 프로파일 (중심 기준 높이 dz → 반경, m). 애셋 메시 히스토그램(scale 0.5) 에서.
_PROFILE_DZ_M = np.array([-0.0331, -0.0315, -0.0284, -0.0252, -0.0221, -0.0190, -0.0159, -0.0127, -0.0096,
                          -0.0065, -0.0033, -0.0002, 0.0030, 0.0060, 0.0092, 0.0123, 0.0155, 0.0186, 0.0217])
_PROFILE_R_M = np.array([0.0000, 0.00985, 0.01295, 0.0145, 0.0158, 0.0166, 0.01805, 0.0194, 0.0212,
                         0.02295, 0.0247, 0.02645, 0.0269, 0.0270, 0.0268, 0.0258, 0.0239, 0.0213, 0.0179])


def fruit_max_radius_m(dz):
    """과실 중심 기준 높이 dz 에서의 최대 반경(넓은 방향). 밑끝 아래는 0."""
    return float(np.interp(dz, _PROFILE_DZ_M, _PROFILE_R_M, left=0.0, right=0.0))


CUP_SIDES = 16
PLATE_MARGIN_M = 0.006       # 바깥 테두리 여유
SKIRT_BOTTOM_WORLD_Z_M = -0.005   # 테이블 상판(z=0) 5mm 아래까지 내려 앉힌다


def fit_origin_m():
    """slot 0 컵 중심 (world, m) — 런 7 과실 6개의 최소자승."""
    acc = []
    for slot, f in FRUIT_REST_M.items():
        r, c = divmod(slot, COLS)
        acc.append(f - r * H_ROW - c * V_COL)
    return np.mean(np.array(acc), axis=0)


def cup_center_local_m(slot):
    r, c = divmod(slot, COLS)
    return r * H_ROW + c * V_COL


def fit_residuals_mm():
    F0 = fit_origin_m()
    return {s: (f - (F0 + cup_center_local_m(s))) * 1000.0 for s, f in FRUIT_REST_M.items()}


def cup_shift_local_m(slot, origin_m):
    """점유 컵의 xy 치우침 = 실측 과실 위치 − 격자 위치. 빈 컵은 0.

    과실은 컵 축에서 최대 ~6mm 치우쳐 놓인다(분면별 접근 기울기 → 매달린 깊이 8.7mm 차, 런 6·7 재현됨).
    행 피치 51.2 < 과실 y 전폭 53.8 이라 림을 키워 흡수할 수 없으므로, 계획서대로 **점유 컵은 실측 과실
    위치(1차 출처)에 맞추고** 빈 9칸만 티칭 격자에 둔다. 격자 대비 ≤6mm 어긋난 컵 6개가 생긴다 — 과실이 실제로
    거기 있으므로 컵이 거기 있는 편이 맞아 보인다.
    """
    if slot not in FRUIT_REST_M:
        return np.zeros(3)
    d = FRUIT_REST_M[slot] - (origin_m + cup_center_local_m(slot))
    return np.array([d[0], d[1], 0.0])


def cup_used_center_local_m(slot, origin_m):
    return cup_center_local_m(slot) + cup_shift_local_m(slot, origin_m)


def cup_rings_local_m(slot, origin_m):
    """컵의 (z_local, rx, ry) 링 목록 — 림에서 바닥까지 CUP_RINGS 개. 중심은 cup_used_center_local_m.

    림은 피치가 허용하는 최대 크기(치우친 컵은 셀 경계까지 남는 만큼)로 고정하고, 그 아래 링은
    **과실 반경 프로파일 + 여유**를 따른다. 바닥은 테이블 상판 위 하한을 적용하므로 낮은 행은 컵이 얕고
    바닥이 넓다 — 과실 몸통이 거기까지 내려와 있기 때문이다.
    """
    c = cup_center_local_m(slot)
    sh = cup_shift_local_m(slot, origin_m)
    half_x = 0.5 * abs(V_COL[0]); half_y = 0.5 * abs(H_ROW[1])
    rim_rx = min(CUP_RIM_RX_M, half_x - abs(sh[0]) - CUP_RIM_EDGE_GAP_M)
    rim_ry = min(CUP_RIM_RY_M, half_y - abs(sh[1]) - CUP_RIM_EDGE_GAP_M)
    rim = c[2] + CUP_RIM_DZ_M
    floor = max(c[2] + CUP_BOTTOM_DZ_M, CUP_FLOOR_MIN_WORLD_Z_M - origin_m[2])
    rings = [(rim, rim_rx, rim_ry)]
    for k in range(1, CUP_RINGS):
        z = rim + (floor - rim) * k / (CUP_RINGS - 1)
        r = fruit_max_radius_m(z - c[2])
        ry = min(rim_ry, max(CUP_BOT_MIN_RY_M, r + CUP_WALL_MARGIN_M))
        rx = min(rim_rx, max(CUP_BOT_MIN_RX_M, FRUIT_XY_RATIO * r + CUP_WALL_MARGIN_M))
        rings.append((z, rx, ry))
    return rings
