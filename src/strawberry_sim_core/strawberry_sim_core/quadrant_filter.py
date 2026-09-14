"""보드 고정 쿼드트리 격자 — 시뮬 비전 모킹용 분면 판정.

[신설 2026-09-09]

실기에서는 세부영역 scan pose 마다 **카메라 시야가 그 분면만 담기 때문에**
`strawberry_fusion_node` 가 그 분면의 딸기만 발행한다. 시뮬에는 카메라가 없어
`fake_vision_node` 가 씬의 모든 딸기를 항상 발행했고, 그 결과
**첫 분면에서 6개를 전부 시도하고 나머지 세 분면은 후보가 없는** 상태가 됐다.
원안 시퀀스(PROJECT_GOAL.md section 1-2) 3/5/6 단계가 성립하지 않는다.

여기서 카메라 시야를 분면 격자로 대체한다. 경계값 처리는
`scan_executor_node._group_poses_by_subcell` 과 **완전히 동일**해야 한다 —
두 곳이 다르면 같은 딸기가 발행은 됐는데 분류는 다른 분면으로 가는 일이 생긴다.

격자 값의 출처 (합성 USD 실측, /World/lab_environment/whiteboard/board):
    x [-495.0, +595.0] mm -> 중심 +50.0 mm
    z [ 265.0, 1055.0] mm -> 중심 +660.0 mm
보드를 옮기면 layout_layer.usd, environment.yaml,
scan_executor_node.BOARD_SUBCELL_*_M 과 함께 여기도 고쳐야 한다.
"""

BOARD_SUBCELL_X_MID_M = 0.050
BOARD_SUBCELL_Z_MID_M = 0.660
# [2026-09-14] 보드 외곽 (scan_executor_node.BOARD_X_MIN_M 등과 같은 값). 세부 칸(깊이 2) 경계에 쓴다.
BOARD_X_MIN_M, BOARD_X_MAX_M = -0.495, 0.595
BOARD_Z_MIN_M, BOARD_Z_MAX_M = 0.265, 1.055

QUADRANTS = ("nw", "ne", "se", "sw")

# cell_id -> 분면. root/nw_flat 은 NW 를 다시 티칭한 변형이라 같은 분면이다.
_CELL_TO_QUADRANT = {
    "root/nw": "nw",
    "root/nw_flat": "nw",
    "root/ne": "ne",
    "root/se": "se",
    "root/sw": "sw",
}


def quadrant_of(x_m: float, z_m: float) -> str:
    """좌표가 속한 분면. scan_executor_node._group_poses_by_subcell 과 동일한 규칙."""
    x_mid, z_mid = BOARD_SUBCELL_X_MID_M, BOARD_SUBCELL_Z_MID_M
    if z_m >= z_mid and x_m <= x_mid:
        return "nw"
    if z_m >= z_mid and x_m > x_mid:
        return "ne"
    if z_m < z_mid and x_m > x_mid:
        return "se"
    return "sw"


def quadrant_from_cell_id(cell_id: str):
    """'root/nw' -> 'nw', 'root/nw/se' -> 'nw'. 분면이 아니면 None(=전체 발행).

    ★ [FIX 2026-09-10] 서브서브셀(root/nw/se 등)을 처리한다.
    scan_executor 는 분면 안을 다시 4등분해 `root/nw/se=SCANNING` 같은 상태도
    발행한다. 종전에는 이 이름이 표에 없어 None -> **필터가 꺼지고 6개 전부 발행**됐다.
    그러면 pick 중 이웃 장애물이 1개에서 5개로 늘어 깊은 파지 오프셋이 막힌다
    (실측 2026-09-09: ripe_01 은 neighbor 1개로 15mm 성공, ripe_02 는 5개로 40mm 로 밀림).
    부모 분면을 그대로 쓰는 것이 맞다 — 서브서브셀도 같은 분면 안이다.

    None 은 **필터를 끄라는 뜻**이다. overview(home) 자세에서의 1차 스캔은
    보드 전체를 보므로 전 분면을 발행하는 것이 맞다.
    """
    cid = (cell_id or "").strip()
    if cid in _CELL_TO_QUADRANT:
        return _CELL_TO_QUADRANT[cid]
    parts = cid.split("/")
    if len(parts) >= 2:
        parent = "/".join(parts[:2])          # root/nw/se -> root/nw
        return _CELL_TO_QUADRANT.get(parent)
    return None


def cell_bounds(cell_id: str):
    """cell_id -> (x0, x1, z0, z1) 또는 None(root/모름 = 전 분면).

    [2026-09-14] 깊이 2 시야. 'root/sw' 는 분면, 'root/sw/se' 는 그 분면을 2×2 로 나눈 세부 칸.
    경계 규칙은 scan_executor_node._quadrant_bounds / _subcell_of_pose 와 같다(부모 분면 중심선).
    """
    parts = (cell_id or "").strip().split("/")
    quad = quadrant_from_cell_id(cell_id)
    if quad is None:
        return None
    xm, zm = BOARD_SUBCELL_X_MID_M, BOARD_SUBCELL_Z_MID_M
    x0, x1 = (BOARD_X_MIN_M, xm) if quad in ("nw", "sw") else (xm, BOARD_X_MAX_M)
    z0, z1 = (zm, BOARD_Z_MAX_M) if quad in ("nw", "ne") else (BOARD_Z_MIN_M, zm)
    sub = parts[2][:2] if len(parts) >= 3 else None
    if sub in QUADRANTS:
        sxm, szm = (x0 + x1) / 2.0, (z0 + z1) / 2.0
        x0, x1 = (x0, sxm) if sub in ("nw", "sw") else (sxm, x1)
        z0, z1 = (szm, z1) if sub in ("nw", "ne") else (z0, szm)
    return (x0, x1, z0, z1)


def in_bounds(x_m: float, z_m: float, bounds) -> bool:
    """bounds 가 None 이면 항상 True. 경계값은 _subcell_of_pose 와 같은 방향으로 처리(x <= 중심, z >= 중심 = 서·북)."""
    if bounds is None:
        return True
    x0, x1, z0, z1 = bounds
    return x0 <= x_m <= x1 and z0 <= z_m <= z1


def parse_cell_state(data: str):
    """'root/nw=SCANNING' -> ('root/nw', 'SCANNING'). 형식이 다르면 (None, None)."""
    if not data or "=" not in data:
        return None, None
    cell_id, _, state = data.partition("=")
    return cell_id.strip(), state.strip()
