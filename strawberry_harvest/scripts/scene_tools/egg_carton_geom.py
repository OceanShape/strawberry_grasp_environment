"""계란판 형상의 단일 출처 (T4-3, 3차). 생성기와 검증기가 공용.

원칙 (2026-09-11 사용자 결정)
  계란판은 **강체 실물**이다 — 수평, 직사각, 컵 15개 동일. 티칭 격자의 왜곡(사이각 84.26°, z 기울기)은
  로봇 티칭 오차이지 컨테이너의 성질이 아니다. 컨테이너를 그 오차에 맞추면 오차가 숨는다.
  컨테이너는 규칙적으로 만들고, 과실은 놓이는 자리에 그대로 둔다 — 과실이 컵에서 벗어나는 만큼이
  티칭 오차의 눈에 보이는 크기다.

3차 (2026-09-11, 실기 영상 정보 반영)
  실기 계란판은 15구(3×5)이고, 실기 런은 과실 3개를 **인접한 칸**(slot 0·1·3 — 티칭 기준점 세 칸 그대로)에
  넣었는데 서로 닿지 않았다. 컨테이너의 **깊이와 구멍 지름**이 이웃 과실을 갈라 놓는다는 뜻이다.
  시뮬 과실은 정지 자세에서 y 전폭 53.8mm 로 행 피치 51.2mm 보다 넓다 — 어떤 컨테이너 형상으로도 이웃 행의
  과실 적도(중심 −2~+13mm 대역)가 겹치는 것 자체는 막을 수 없다(물리 없음, 과실은 놓인 자리에 고정). 대신
  **림을 그 대역보다 위(중심 +20mm)에 두면** 겹침은 컵 벽 안쪽에 숨고 밖에서는 어깨와 줄기만 보인다 — 실기
  영상과 같은 화면이다. 구멍은 피치가 허용하는 최대(림 rx27/ry25, 능선 5.8/1.2mm). 과실은 키네마틱이라 바닥에
  기대지 않으므로 깊이는 자유다 — 바닥은 과실 밑끝 바로 아래(z 1)에 두고 림만 올렸다(깊이 21 → 56mm).
  추가로 계란판의 **수평 중점을 테이블 중심축(y=0)** 에 맞췄다 — 2차는 −10.8mm 옆에 있었다. 계란판은 배치 격자에
  합동으로 따라가므로 격자 자체를 y 로 옮겨야 하고, 플래너 파라미터 `taught_grid_shift_y_m`(기본 0, 시뮬 +0.0108)
  로 분리했다 (SUBMISSION_PLAN §2 부수규칙 1). 값의 출처는 아래 GRID_SHIFT_Y_M.

4차 (2026-09-11, 런 8 화면 관찰 + 사용자 요구 4건)
  런 8(계란판 3차)에서 사용자가 옆의 빈 컵으로 들여다보니 과실 적도가 벽을 뚫고 나와 보였다 — 3차의 "림 아래라 안 보인다"는
  틀렸다(벽은 양면 렌더, 빈 컵 안쪽에서 보인다). 요구: ① 구멍은 과실의 수평 최대 지름을 담게 ② 컵 바닥은 놓인 과실 밑끝에
  딱 맞는 높이·형태로 ③ 계란판 바닥은 테이블 상판에 닿게 ④ 슬롯 시퀀스를 바꿔서라도 **인접 칸**에 배치.
  ①은 피치를 바꾸지 않고는 불가능하다(과실 y 전폭 53.8 + 착지 산포 > 실기 행 피치 51.2). 실기 컵에는 실기 모형 과실이 인접
  3칸에 들어갔으므로 이것은 **과실 애셋 치수의 불일치**다. 실기 상수는 그대로 두고 시뮬 전용 파라미터
  `taught_grid_pitch_override_m`(기본 0 = 실기 피치; 시뮬 0.068) 로 배치 격자를 **정사각 68mm** 로 바꾸고 컵 격자도 같게 했다.
  컵 안쪽은 **과실 프로파일에서 유도**한다: 높이별 (hx, hy) + 그 높이에 걸리는 과실들의 착지 치우침 + 여유 1.5mm, 위로 갈수록
  줄지 않게(오버행 금지). 바닥 = 가장 낮게 놓인 과실 밑끝 높이(런 8 실측 z 0.8mm). 계란판 밑변 = 테이블 상판(z 0, 밑면 없음).
  착지 예측은 **수확 순서**로 매달림 변위를 재배정한다 — 런 8 은 슬롯 0,1,6,7,12,13 에 놓았고 4차 시퀀스는 0,1,3,4,6,7 이다.

좌표계
  애셋 프레임 = 계란판 자체 프레임. 원점 = slot 0 컵 중심의 바로 아래 테이블 상판(z=0).
  열(slot%3) 은 -x, 행(slot//3) 은 -y 방향 (티칭 격자와 같은 방향 관례). 컵 15개는 축 정렬 직교 격자.
  씬 배치(layout_layer.usd) = 아래 fit 의 (translate, z축 회전).

피치
  열 |slot1-slot0| = 59.8mm, 행 |slot3-slot0| = 51.2mm — 플래너 상수의 **크기**만 쓴다. 각도는 쓰지 않는다.
  (컨테이너 실측 치수는 기록이 없다. 3점 티칭 오차는 각도에 주로 들어가고 거리에는 덜 들어간다고 본다.)

자세 fit
  플래너가 겨냥하는 ee 격자 15점 + 평균 ee→과실 변위(런 7 실측 6개) 를 목표로 격자를 맞춘다.
  2026-09-11 사용자 결정: 계란판은 베이스 축 정렬(yaw 0), 배치 격자도 플래너 파라미터
  orthogonalize_taught_grid=true 로 직교화 — 두 격자가 합동이라 잔차는 과실별 매달린 깊이 차뿐이다.
  (직교화 전의 티칭 격자에 맞출 때는 Kabsch 가 왜곡을 양쪽 축으로 나눠 가져 yaw -7.24° 가 나왔다.)
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
REAL_PITCH_COL_M = float(np.linalg.norm(V_COL_TAUGHT[:2]))          # 0.0598 (실기 티칭)
REAL_PITCH_ROW_M = float(np.linalg.norm(H_ROW_TAUGHT[:2]))          # 0.0512 (실기 티칭)
# 4차: 시뮬 전용 정사각 피치 = 플래너 파라미터 taught_grid_pitch_override_m (run_nodes.sh). 근거는 머리말 4차.
#   필요 최소 = 2·(과실 y 반폭 26.9 + 착지 |dy| ≤ 3.8 + 여유 1.5) + 벽 ≥ 2 = 66.4 → 68 (런 간 산포 여유 포함).
PITCH_M = 0.068
PITCH_COL_M = PITCH_M
PITCH_ROW_M = PITCH_M

# 1차 출처: 런 9 (20260911T034247-b358aa9c) Kit 로그 `[bridge] RELEASE ... frozen at` (m). 수확 순서 = 아래 dict 순서.
#   런 9 격자 = 4차 그대로: 정사각 68mm + y +45.2mm — 매달림 변위는 그 격자의 겨냥 ee 에 대해 계산한다 (ee_run_m).
RUN_ID = "20260911T034247-b358aa9c"
RUN_SLOT_SEQUENCE = [0, 1, 3, 4, 6, 7]
RUN_GRID = dict(pitch_col=0.068, pitch_row=0.068, shift_y=0.0452)
FRUIT_REST_M = {                              # 런 9 슬롯 → 과실 중심 (world). 과실 prim: ripe_01, 02, 03, 05, 06, 04 순
    0: np.array([0.7673, 0.1332, 0.0345]),
    1: np.array([0.6981, 0.1351, 0.0352]),
    3: np.array([0.7663, 0.0696, 0.0358]),
    4: np.array([0.7104, 0.0688, 0.0354]),
    6: np.array([0.7739, 0.0024, 0.0337]),
    7: np.array([0.7060, 0.0022, 0.0355]),
}
# (이력) 런 8 20260911T025246-941542d8 (직교화 실기 피치 59.8×51.2, shift 0.0108, 시퀀스 0,1,6,7,12,13):
#        0 (0.7678,0.0978,0.0363) 1 (0.7060,0.1005,0.0354) 6 (0.7658,0.0002,0.0345) 7 (0.7146,-0.0002,0.0351)
#        12 (0.7743,-0.1014,0.0338) 13 (0.7151,-0.1016,0.0362). 런 8 변위로 예측한 4차 착지와 런 9 실측의 차: x ≤ 4.0(slot 4), y ≤ 1.0, z ≤ 1.8mm.
# (이력) 런 7 20260910T140636-71389d3a: 0 (0.7683,0.0883,0.0365) 1 (0.7066,0.0946,0.0372) 6 (0.7505,-0.0087,0.0310)
#        7 (0.7012,-0.0052,0.0308) 12 (0.7425,-0.1095,0.0242) 13 (0.6821,-0.1066,0.0256) — 격자 직교화 전, 평행이동 0
# 4차 배치 시퀀스 (run_nodes.sh taught_slot_sequence). 수확 i번째 과실이 PLACE_SLOT_SEQUENCE[i] 에 놓인다.
PLACE_SLOT_SEQUENCE = [0, 1, 3, 4, 6, 7]

# 과실 정지 자세: 파지 자세에서 수직축 기준 ~82° 회전(장축은 수직 유지). 중심 기준 밑끝 -33.0mm, 줄기끝 +29.5mm.
# 높이별 반폭 (mm 단위 2mm 구간, strawberry_ripe_01 메쉬 ×0.005 를 z −81.5° 회전 — verify 와 같은 규약).
# hx = 열(x) 방향 반폭, hy = 행(y) 방향 반폭. 넓은 쪽(y)이 행 피치 51.2 의 반 25.6 을 넘는 대역은 dz −1~+13mm.
FRUIT_BOTTOM_BELOW_CENTER_M = 0.0330
FRUIT_TOP_ABOVE_CENTER_M = 0.0295
_PROFILE_DZ_M = np.array([-32, -30, -28, -26, -24, -22, -20, -18, -16, -14, -12, -10, -8, -6, -4, -2, 0,
                          2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30]) / 1000.0
_PROFILE_HX_M = np.array([5.5, 7.9, 10.1, 11.0, 12.1, 13.1, 14.1, 14.8, 16.1, 16.8, 18.0, 18.7, 19.5, 20.4, 20.8,
                          21.1, 21.6, 21.9, 22.0, 22.3, 22.0, 22.0, 21.0, 20.3, 19.6, 17.8, 15.5, 15.9, 15.4,
                          16.7, 1.0, 0.8]) / 1000.0
_PROFILE_HY_M = np.array([7.2, 10.9, 12.5, 14.2, 15.0, 15.4, 16.3, 16.6, 17.6, 18.1, 19.4, 20.2, 21.4, 22.9, 23.8,
                          25.2, 26.0, 26.5, 26.9, 26.9, 26.9, 26.5, 25.6, 24.5, 23.1, 21.1, 19.6, 16.7, 14.3,
                          13.7, 2.9, 2.5]) / 1000.0


def fruit_half_widths_m(dz):
    """과실 중심 기준 높이 dz 에서의 (hx, hy) 반폭. 과실 밖은 0."""
    hx = float(np.interp(dz, _PROFILE_DZ_M, _PROFILE_HX_M, left=0.0, right=0.0))
    hy = float(np.interp(dz, _PROFILE_DZ_M, _PROFILE_HY_M, left=0.0, right=0.0))
    return hx, hy


# 컵 (15개 동일, 애셋 프레임 z = 테이블 상판 기준) — 4차: 과실 프로파일에서 유도. 실제 계산은 아래 _build_cup() (착지 예측 뒤).
PLATE_TOP_Z_M = 0.057        # 판 윗면 = 림 높이 (3차 그대로 — 과실 적도 위, 어깨·줄기만 보인다)
SKIRT_BOTTOM_Z_M = 0.0       # 계란판 밑변 = 테이블 상판. 밑면은 만들지 않는다 (상판과 겹쳐 떨림).
CUP_CLEARANCE_M = 0.0015     # 컵 벽 – (과실 표면 + 착지 치우침) 여유
CUP_WALL_MIN_M = 0.002       # 이웃 컵 림 사이 능선 최소 폭 (verify 게이트)
CUP_SIDES = 16
PLATE_MARGIN_M = 0.006
CUP_RING_DZ_M = [0.0, 0.0015, 0.0035, 0.006, 0.009, 0.0125, 0.016, 0.020, 0.024, 0.028, 0.032, 0.036, 0.040, 0.044, 0.048]  # 바닥 기준 링 높이

# 자세 각도 강제. None = 티칭 격자에 최소자승 fit (-7.24°, 컵 중심 잔차 최대 5.3mm).
#   0.0   = 로봇 베이스 축에 정렬 — 잔차 최대 16.7mm (행 0·4 과실이 x 로 ±16mm, 능선 위에 걸린다)
#   -3.30 = 티칭 열 축에 정렬 — 잔차 최대 10.3mm
YAW_OVERRIDE_DEG = 0.0      # 2026-09-11 사용자 결정: 베이스 축 정렬. 배치 격자도 직교화하므로 잔차는 매달린 깊이 차뿐

# 플래너 파라미터 taught_grid_shift_y_m 과 같은 값 (run_nodes.sh 가 준다). 격자 전체를 world y 로 평행이동.
#   출처: 계란판 중점 y = (slot 0 ee y + 평균 ee→과실 변위 y) − 2·행피치 → 0 이 되게: 2·68 − 52.39 − 38.4 = +45.2mm (런 8 기준; 런 9 ideal 44.65, 차 0.55mm 라 유지)
#   (3차는 실기 행피치 51.2 로 +10.8 이었다.) ideal_grid_shift_y_m() 이 현재 실측으로 다시 계산한다 —
#   FRUIT_REST_M 갱신 뒤 1mm 넘게 달라지면 이 상수와 run_nodes.sh 를 같이 고친다 (verify 가 대조).
GRID_SHIFT_Y_M = 0.0452
TABLE_AXIS_Y_M = 0.0         # 테이블(/World/lab_environment/table, y ±300mm) 의 중심축


def cup_center_asset_m(slot):
    """애셋 프레임 컵 중심 (xy, z=0). 열은 -x, 행은 -y."""
    r, c = divmod(slot, COLS)
    return np.array([-c * PITCH_COL_M, -r * PITCH_ROW_M, 0.0])


def carton_center_asset_m():
    """컵 격자의 수평 중점 (애셋 프레임) = 컵 15개 중심의 평균."""
    return np.mean([cup_center_asset_m(s) for s in range(ROWS * COLS)], axis=0)


# 플래너 파라미터 orthogonalize_taught_grid=true + taught_grid_pitch_override_m=PITCH_M + taught_grid_shift_y_m 와 같은 규칙.
PLACEMENT_ORTHOGONALIZED = True


def ee_taught_m(slot):
    """실기 티칭 격자의 ee 위치 (플래너 기본값 — 직교화 false / 피치 0 / shift 0)."""
    r, c = divmod(slot, COLS)
    return _EE0 + r * H_ROW_TAUGHT + c * V_COL_TAUGHT


def ee_grid_m(slot, pitch_col, pitch_row, shift_y):
    """직교화된 격자 (축 -x/-y, z 수평) + y 평행이동. tray_place_policy.taught_grid_slot_offset_m 와 같은 식."""
    r, c = divmod(slot, COLS)
    return _EE0 + np.array([-c * pitch_col, -r * pitch_row + shift_y, 0.0])


def ee_run_m(slot):
    """실측 런(RUN_ID)이 겨냥한 ee — 매달림 변위의 기준."""
    return ee_grid_m(slot, RUN_GRID["pitch_col"], RUN_GRID["pitch_row"], RUN_GRID["shift_y"])


def ee_placed_m(slot):
    """지금 run_nodes.sh 설정(정사각 PITCH_M + GRID_SHIFT_Y_M)으로 플래너가 겨냥하는 ee."""
    if not PLACEMENT_ORTHOGONALIZED:
        return ee_taught_m(slot) + np.array([0.0, GRID_SHIFT_Y_M, 0.0])
    return ee_grid_m(slot, PITCH_COL_M, PITCH_ROW_M, GRID_SHIFT_Y_M)


def fruit_hang_offsets_m():
    """수확 i번째 과실의 ee→과실 변위 d_i (world) = 런 실측 착지 − 그 런의 겨냥 ee. 분면별 매달린 깊이 차가 여기 들어 있다.
    변위는 그리퍼–과실 상대자세의 성질이라 슬롯과 무관하다고 본다 → 다른 시퀀스의 착지 예측에 재사용."""
    return [FRUIT_REST_M[s] - ee_run_m(s) for s in RUN_SLOT_SEQUENCE]


def predicted_landings():
    """[(slot, 과실 중심 world)] — 4차 시퀀스로 놓았을 때의 예상 착지 = 겨냥 ee + 수확 순서가 같은 과실의 실측 변위."""
    d = fruit_hang_offsets_m()
    return [(slot, ee_placed_m(slot) + d[i]) for i, slot in enumerate(PLACE_SLOT_SEQUENCE)]


def predicted_fruit_rest_m(slot):
    for s, f in predicted_landings():
        if s == slot:
            return f
    raise KeyError(slot)


def ideal_grid_shift_y_m():
    """현재 FRUIT_REST_M 기준으로 계란판 중점 y 를 TABLE_AXIS_Y_M 에 올리는 평행이동량 (GRID_SHIFT_Y_M 의 근거)."""
    mean_off = np.mean(fruit_hang_offsets_m(), axis=0)
    center_y_unshifted = _EE0[1] + mean_off[1] + carton_center_asset_m()[1]
    return float(TABLE_AXIS_Y_M - center_y_unshifted)


def _taught_targets_m():
    """플래너가 겨냥하는 ee 격자 15점 + 평균 ee→과실 변위 (xy). fit 의 목표."""
    mean_off = np.mean(fruit_hang_offsets_m(), axis=0)
    return {s: (ee_placed_m(s) + mean_off)[:2] for s in range(15)}, mean_off


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


def carton_center_world_m(translate, yaw):
    """씬 배치(translate, yaw)에서의 컵 격자 수평 중점 (world)."""
    c = carton_center_asset_m()
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([translate[0] + cy * c[0] - sy * c[1], translate[1] + sy * c[0] + cy * c[1], 0.0])


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



def _build_cup():
    """컵 안쪽 (z, rx, ry) 링 — 림에서 바닥 순. 과실 프로파일 + 착지 치우침 + 여유, 위로 갈수록 줄지 않게.
    바닥 = 가장 낮게 놓인 과실의 밑끝 높이(0.1mm 단위, 0 이상). 착지 치우침은 fit 으로 놓인 컵 중심 기준."""
    t, yaw, _ = fit_pose()
    land = []
    for slot, f in predicted_landings():
        c = t + cup_center_asset_m(slot)
        land.append((f[2], f[0] - c[0], f[1] - c[1]))
    floor = max(0.0, np.floor(min(cz - FRUIT_BOTTOM_BELOW_CENTER_M for cz, _, _ in land) * 1e4) / 1e4)
    zs = [floor + dz for dz in CUP_RING_DZ_M if floor + dz < PLATE_TOP_Z_M - 0.002] + [PLATE_TOP_Z_M]
    rings, rx_run, ry_run = [], 0.0, 0.0
    for z in zs:
        rx = max(fruit_half_widths_m(z - cz)[0] + abs(dx) for cz, dx, dy in land) + CUP_CLEARANCE_M
        ry = max(fruit_half_widths_m(z - cz)[1] + abs(dy) for cz, dx, dy in land) + CUP_CLEARANCE_M
        rx_run, ry_run = max(rx_run, rx), max(ry_run, ry)          # 오버행 금지
        rings.append((round(z, 5), round(rx_run, 4), round(ry_run, 4)))
    return floor, rings[::-1]


CUP_FLOOR_Z_M, CUP_RINGS_M = _build_cup()
