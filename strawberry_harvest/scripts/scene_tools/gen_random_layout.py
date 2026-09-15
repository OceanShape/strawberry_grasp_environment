#!/usr/bin/env python3
"""T4d 무작위 과실 배치 생성기 — 시드 하나 = 배치 하나. 세 파일을 한 번에 쓴다.

    python3 strawberry_harvest/scripts/scene_tools/gen_random_layout.py --snapshot-base   # 지금 배치를 base_layout.json 으로 (최초 1회)
    python3 strawberry_harvest/scripts/scene_tools/gen_random_layout.py --seed 7 --apply  # 시드 7 배치를 씬에 적용
    python3 strawberry_harvest/scripts/scene_tools/gen_random_layout.py --seed 7          # 적용 없이 배치·기대값만 출력
    python3 strawberry_harvest/scripts/scene_tools/gen_random_layout.py --restore         # base_layout.json 으로 되돌림 (T5 녹화 전 필수)

리포 루트에서 실행한다. Isaac 없이 텍스트만 바꾼다(씬 재로드는 별도).

왜 세 파일인가 (docs/random_layout_audit.md A1~A3): 과실 translate(layout_layer) · 덩굴 translate(layout_layer, 같은 값) ·
줄기 FixedJoint localPos0(physics_layer) 가 같은 좌표를 세 번 적는 구조다. 손으로 맞추던 것을 여기서 한 번에 쓴다.

무엇을 섞나: 과실 12개(익은 8 · 미숙 4, 이름 고정)의 **x·z 만**. y 는 0.7828 고정(보드에 매단 구조, audit B7).

표본 범위와 거부 조건 (2026-09-15 결정, SUBMISSION_PLAN T4d):
  - x ∈ [X_MIN, X_MAX] = ±0.42 — 실기 노드 DIRECT_GRASP_TARGET_X_RANGE_M(±0.45, audit B1) 안쪽. 실기가 대상에서 빼는 띠에 과실을 두면
    시퀀스가 아니라 그 파라미터를 재는 것이라 뺐다. 보드 자체는 x −0.495~+0.595.
  - z ∈ [Z_MIN, Z_MAX] = 0.35~0.90 — 런 12~14 배치가 검증된 높이 범위(0.378~0.898). 보드 z 0.265~1.055 의 나머지는 티칭 자세 도달성 미검증.
  - 중심 간격 ≥ MIN_SPACING(0.075) — 과실 폭 0.054 + 여유. 실행기 중복 제거 30mm(B8) 는 자동 충족.
  - 덩굴 통로: 다른 과실 중심이 |dx| < CORRIDOR_HALF_W 이고 0 < dz < CORRIDOR_H 이면 거부 — 덩굴(과실 위 +25~+82mm, 보드로 휘어 올라감)이
    위 과실을 뚫는 배치. 적용 뒤 verify_vines.py [5] 로 실제 메시 간격을 다시 잰다.
  - 분면 경계(x 0.050, z 0.660)에서 BOUNDARY_MARGIN(0.015) 띄움 — 경계 위 과실은 분면·세부 칸 판정이 흔들린다(B9).
그 밖의 실기 정책 분기(x>0.25 오른쪽 사다리, 스윙 가드, 이송 거부 …)는 일부러 제약하지 않는다 — 그게 측정 대상이다.

출력: log/m3/random/layouts/seed_<N>.json — 좌표, 분면별 익은 수(기대 OVERVIEW_SCAN), 분할 기대(후보 ≥3 인 분면), 최소 간격.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import re
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
LAYOUT = os.path.join(REPO, "strawberry_harvest/scenes/layers/layout_layer.usd")
PHYSICS = os.path.join(REPO, "strawberry_harvest/scenes/layers/physics_layer.usd")
OUT_DIR = os.path.join(REPO, "log/m3/random/layouts")
BASE_JSON = os.path.join(OUT_DIR, "base_layout.json")

FRUITS = ["ripe_01", "ripe_02", "ripe_03", "ripe_04", "ripe_05", "ripe_06", "ripe_07", "ripe_08",
          "unripe_02", "unripe_04", "unripe_05", "unripe_06"]
Y_FIXED = 0.7828

X_MIN, X_MAX = -0.42, 0.42
Z_MIN, Z_MAX = 0.35, 0.90
MIN_SPACING = 0.075
CORRIDOR_HALF_W = 0.045
CORRIDOR_H = 0.12
BOUNDARY_MARGIN = 0.015
X_MID, Z_MID = 0.050, 0.660          # quadrant_filter.BOARD_SUBCELL_X_MID_M / Z_MID_M
SUBDIVIDE_MIN = 3                    # run_nodes.sh subdivide_min_candidates

_TR = r"double3 xformOp:translate = \(([-0-9.eE+]+), ([-0-9.eE+]+), ([-0-9.eE+]+)\)"


def _block_re(prefix: str, name: str) -> re.Pattern:
    # over "strawberry_ripe_01" ... { ... double3 xformOp:translate = (...) ... }
    return re.compile(r'(over "%s%s"[^\n]*\n\s*\{[^}]*?)%s' % (prefix, name, _TR), re.S)


def read_layout() -> dict:
    s = open(LAYOUT, encoding="utf-8").read()
    out = {}
    for f in FRUITS:
        m = _block_re("strawberry_", f).search(s)
        if not m:
            sys.exit("layout_layer.usd 에서 strawberry_%s 를 못 찾았다" % f)
        out[f] = [float(m.group(2)), float(m.group(3)), float(m.group(4))]
    return out


def quadrant(x: float, z: float) -> str:
    return ("n" if z >= Z_MID else "s") + ("w" if x < X_MID else "e")


def expectations(pos: dict) -> dict:
    ripe = {f: p for f, p in pos.items() if not f.startswith("unripe")}
    counts = {q: 0 for q in ("nw", "ne", "se", "sw")}
    for f, p in ripe.items():
        counts[quadrant(p[0], p[2])] += 1
    pts = list(pos.values())
    dmin = min(math.dist((a[0], a[2]), (b[0], b[2])) for i, a in enumerate(pts) for b in pts[i + 1:])
    return {
        "overview_counts": counts,
        "pruned": [q for q, c in counts.items() if c == 0],
        "expect_subdivide": [q for q, c in counts.items() if c >= SUBDIVIDE_MIN],
        "min_center_spacing_m": round(dmin, 4),
    }


def sample(seed: int, max_tries: int = 200000) -> dict:
    rng = random.Random(seed)
    placed: list[tuple[float, float]] = []
    tries = 0
    while len(placed) < len(FRUITS):
        tries += 1
        if tries > max_tries:
            sys.exit("시드 %d: %d 회 안에 배치를 못 찾았다 — 제약을 확인할 것" % (seed, max_tries))
        x = rng.uniform(X_MIN, X_MAX)
        z = rng.uniform(Z_MIN, Z_MAX)
        if abs(x - X_MID) < BOUNDARY_MARGIN or abs(z - Z_MID) < BOUNDARY_MARGIN:
            continue
        ok = True
        for (px, pz) in placed:
            if math.dist((x, z), (px, pz)) < MIN_SPACING:
                ok = False
                break
            dx, dz = x - px, z - pz
            # 새 과실이 기존 과실의 덩굴 통로 안(위쪽) 또는 기존 과실이 새 과실의 통로 안
            if abs(dx) < CORRIDOR_HALF_W and (0 < dz < CORRIDOR_H or 0 < -dz < CORRIDOR_H):
                ok = False
                break
        if ok:
            placed.append((x, z))
    # 이름 순서 고정, 위치만 섞는다 (audit D: 익음은 이름으로 판정)
    return {f: [round(x, 4), Y_FIXED, round(z, 4)] for f, (x, z) in zip(FRUITS, placed)}


def apply(pos: dict) -> None:
    s = open(LAYOUT, encoding="utf-8").read()
    for f, p in pos.items():
        tr = "double3 xformOp:translate = (%s, %s, %s)" % (_fmt(p[0]), _fmt(p[1]), _fmt(p[2]))
        for prefix in ("strawberry_", "vine_"):
            s, n = _block_re(prefix, f).subn(lambda m, tr=tr: m.group(1) + tr, s, count=1)
            if n != 1:
                sys.exit("layout_layer.usd %s%s translate 치환 실패" % (prefix, f))
    open(LAYOUT, "w", encoding="utf-8").write(s)

    s = open(PHYSICS, encoding="utf-8").read()
    for f, p in pos.items():
        pat = re.compile(r'(def PhysicsFixedJoint "stem_%s"[^{]*\{[^}]*?point3f physics:localPos0 = )\([^)]*\)' % f, re.S)
        s, n = pat.subn(lambda m, p=p: m.group(1) + "(%s, %s, %s)" % (_fmt(p[0]), _fmt(p[1]), _fmt(p[2])), s, count=1)
        if n != 1:
            sys.exit("physics_layer.usd stem_%s localPos0 치환 실패" % f)
    open(PHYSICS, "w", encoding="utf-8").write(s)


def _fmt(v: float) -> str:
    return ("%.4f" % v).rstrip("0").rstrip(".") if abs(v) >= 1e-9 else "0"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int)
    ap.add_argument("--apply", action="store_true", help="씬 파일 3곳에 쓴다 (씬 재로드는 별도)")
    ap.add_argument("--restore", action="store_true", help="base_layout.json 으로 되돌린다")
    ap.add_argument("--snapshot-base", action="store_true", help="지금 씬 배치를 base_layout.json 으로 저장")
    a = ap.parse_args()
    os.makedirs(OUT_DIR, exist_ok=True)

    if a.snapshot_base:
        pos = read_layout()
        json.dump({"seed": "base", "positions": pos, "expect": expectations(pos)},
                  open(BASE_JSON, "w", encoding="utf-8"), indent=1, ensure_ascii=False)
        print("base_layout.json <-", {f: p for f, p in pos.items()})
        return 0
    if a.restore:
        if not os.path.exists(BASE_JSON):
            sys.exit("base_layout.json 이 없다 — 먼저 --snapshot-base")
        pos = json.load(open(BASE_JSON, encoding="utf-8"))["positions"]
        apply(pos)
        print("restored base layout (12 fruits) -> layout_layer.usd / physics_layer.usd. 씬 재로드 필요.")
        return 0
    if a.seed is None:
        ap.error("--seed N 또는 --restore / --snapshot-base")
    if a.apply and not os.path.exists(BASE_JSON):
        sys.exit("base_layout.json 이 없다 — 적용 전에 --snapshot-base 로 지금 배치를 보존할 것")

    pos = sample(a.seed)
    exp = expectations(pos)
    rec = {"seed": a.seed, "positions": pos, "expect": exp,
           "constraints": {"x": [X_MIN, X_MAX], "z": [Z_MIN, Z_MAX], "min_spacing_m": MIN_SPACING,
                           "corridor_half_w_m": CORRIDOR_HALF_W, "corridor_h_m": CORRIDOR_H,
                           "boundary_margin_m": BOUNDARY_MARGIN, "y_m": Y_FIXED}}
    path = os.path.join(OUT_DIR, "seed_%d.json" % a.seed)
    json.dump(rec, open(path, "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    for f, p in pos.items():
        print("  %-10s x %+.3f  z %.3f  %s" % (f, p[0], p[2], quadrant(p[0], p[2])))
    print("expect: OVERVIEW_SCAN %s  pruned %s  subdivide %s  min spacing %.1fmm" % (
        " ".join("%s:%d" % (q, exp["overview_counts"][q]) for q in ("nw", "ne", "se", "sw")),
        exp["pruned"], exp["expect_subdivide"], exp["min_center_spacing_m"] * 1000))
    print("saved", os.path.relpath(path, REPO))
    if a.apply:
        apply(pos)
        print("applied seed %d -> layout_layer.usd / physics_layer.usd. 씬 재로드 필요." % a.seed)
    return 0


if __name__ == "__main__":
    sys.exit(main())
