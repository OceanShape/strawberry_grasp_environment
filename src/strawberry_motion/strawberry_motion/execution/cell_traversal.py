"""쿼드트리 세부영역 순회 순서 해석 — scan_executor 보조 모듈.

[신설 2026-09-09]

원안 시퀀스(PROJECT_GOAL.md §1-2)의 6단계는 "4개 영역 전부 수확하고 종료" 다. 원본 실행기의
순서목록과 이 리포 YAML 을 **함께 돌리면** NW 가 빠질 수 있었다. 이유는 이름 불일치다:

    원본 _ALL_CELLS_ZORDER = ["root/sw", "root/nw_flat", "root/ne", "root/se"]   (데모 촬영용)
    이 리포 YAML cell_id   =  root/sw,   root/nw,        root/ne,   root/se
                                         ^^^^^^^^^^^^ YAML 에 없어서 교집합에서 탈락

원본 코드는 `[c for c in 순서목록 if c in targets]` 라 없는 이름을 **조용히 버렸다.**
NW 가 통째로 빠져도 로그 한 줄 남지 않는 구조였다.

[09-17 정정] 이건 코드 조합의 결함이지 실기 동작이 아니다. nw_flat 관절값은 원 팀이 티칭해 뒀고
(`_baseline/.../joint_jog_control.py` NAMED_POSES — 실기 YAML 자체는 이 PC 에 없어 미확인), 원 팀 기록(민1 STEP 6·민2 §7)은
nw -> ne -> se -> sw 4셀 순회 성공을 적는다. 3분면만 돈 런 로그는 이 리포에도 0건이다
(portfolio/H_scope_decisions.md §11). 종전 문구 "3개 영역만 순회하고 있었다"는 관측이 아니었다.
순서목록은 09-09 에 원 팀 기록 순서로 되돌렸고, 09-17 에 `_ALL_CELLS_CLOCKWISE_ORDER` 로 이름을 바꿨다
(Z-order 가 아니라 시계 방향이라서).

여기서는 (1) 별칭을 해석해 YAML 에 실재하는 이름으로 바꾸고, (2) 네 분면 중 자세가
없는 것이 있으면 **호출자가 경고할 수 있도록 돌려준다.** 조용히 빠지는 일이 없게 한다.
"""

QUADRANTS = ("nw", "ne", "se", "sw")

# 같은 분면을 가리키는 이름들. 앞에 오는 것을 우선 채택한다.
# root/nw_flat 은 NW 를 평평한(occlusion 적은) 자세로 다시 티칭한 변형이다.
CELL_ALIASES = {
    "nw": ("root/nw_flat", "root/nw"),
    "ne": ("root/ne",),
    "se": ("root/se",),
    "sw": ("root/sw",),
}


def quadrant_of(cell_id: str):
    """'root/nw_flat' -> 'nw'. 어느 분면도 아니면 None."""
    for quad, names in CELL_ALIASES.items():
        if cell_id in names:
            return quad
    return None


def resolve_traversal_order(preferred_order, available_cells):
    """순회 순서를 YAML 에 실재하는 cell_id 로 해석한다.

    preferred_order : 원하는 순서의 cell_id 목록 (scan_executor_node._ALL_CELLS_CLOCKWISE_ORDER)
    available_cells : YAML 에서 읽은 cell_id 들 (dict 또는 set)

    반환 (order, missing_quadrants)
      order              — 실제로 이동 가능한 cell_id 목록, 요청 순서 유지
      missing_quadrants  — 자세가 하나도 없는 분면 이름들 ('nw' 등)
    """
    available = set(available_cells)
    order = []
    seen = set()

    def _take(quad):
        for name in CELL_ALIASES[quad]:
            if name in available and name not in seen:
                seen.add(name)
                order.append(name)
                return True
        return False

    # 1) 요청 순서를 먼저 따른다 (별칭 해석 포함)
    for cell_id in preferred_order:
        quad = quadrant_of(cell_id)
        if quad is None:
            if cell_id in available and cell_id not in seen:
                seen.add(cell_id)
                order.append(cell_id)
            continue
        _take(quad)

    # 2) 요청 목록에 없던 분면도 자세가 있으면 뒤에 붙인다 — 4분면 전부 도는 것이 원안이다.
    for quad in QUADRANTS:
        if not any(quadrant_of(c) == quad for c in order):
            _take(quad)

    missing = [q for q in QUADRANTS
               if not any(quadrant_of(c) == q for c in order)]
    return order, missing
