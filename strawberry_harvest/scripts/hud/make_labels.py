#!/usr/bin/env python3
"""HUD 의 한글 라벨을 PNG 로 미리 그린다 — Kit 이 한글 글리프를 못 그리기 때문.

Isaac Sim 5.1 의 텍스트 렌더러는 한글을 '?' 로 찍는다 (스크립트 에디터조차 그렇다).
style 의 "font" 로 한글 폰트를 넘겨도 마찬가지였다. 그래서 글자는 Kit 이 아니라
여기서 Pillow 로 그려 두고, HUD 는 ui.Image 로 붙인다. 숫자는 ASCII 라 그대로 Label.

실행:  python3 make_labels.py         (Pillow + 시스템 Noto Sans CJK 필요)
출력:  labels/<이름>.png  +  labels/manifest.json (이름 -> 표시 폭/높이)

트리 패널 문구(방향·분할·제외)는 tree_model.py 가 단일 출처다 (2026-09-11).
"""
import json
import os
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tree_model  # noqa: E402

SRC = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FACE_KR = 1
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "labels")
SCALE = 2          # HiDPI 대비 2배로 그려서 절반 크기로 표시한다

# (이름, 글자, 크기px, 색, 볼드)  — 크기·색은 ../isaac_sim_viewport_display.py 와 같은 값
TEXT = _rgb = lambda h: ((h >> 16) & 255, (h >> 8) & 255, h & 255, 255)
C_TEXT, C_DIM = _rgb(0xE8EDF5), _rgb(0x7B8494)
C_OK = _rgb(0x5AD469)    # isaac_sim_viewport_display.C_OK — 배치 단계·완료와 같은 초록
C_BAD = _rgb(0xFF4D5E)   # isaac_sim_viewport_display.C_BAD — 노드 램프 빨강과 같은 값 (빨강은 실패 전용)
PHASE = {"IDLE": 0x7B8494, "SCAN_MOVE": 0x4A9EFF, "DETECT": 0x00C8C8, "PLAN": 0xA78BFA,
         "APPROACH": 0xFFB547, "ENTER": 0xFFB547, "GRASP": 0xFF7A3D,
         "DETACH": 0xFF3D5C, "RETREAT": 0x4A9EFF,
         "PLACE": 0x5AD469, "RETURN": 0x4A9EFF, "DONE": 0x5AD469}
STATE_KO = {"IDLE": "대기", "SCAN_MOVE": "스캔 이동", "DETECT": "타겟 수신", "PLAN": "경로 계산",
            "APPROACH": "접근", "ENTER": "진입", "GRASP": "하강 + 파지",
            "DETACH": "분리", "RETREAT": "후퇴",
            "PLACE": "배치", "RETURN": "복귀", "DONE": "완료"}

REGION_KO = {"home": "HOME", "nw": "북서 NW", "ne": "북동 NE",
             "sw": "남서 SW", "se": "남동 SE"}

ITEMS = [
    ("head_nodes", "노드", 15, C_DIM, False),
    ("head_region", "영역", 15, C_DIM, False),
    ("head_targets", "타겟", 15, C_DIM, False),
    ("head_placed", "배치", 15, C_DIM, False),
    ("head_phase", "단계", 15, C_DIM, False),
    ("head_tree", "트리", 15, C_DIM, False),
    ("node_vision", "인식", 15, C_TEXT, False),
    ("node_planner", "플래너", 15, C_TEXT, False),
    ("node_controller", "제어", 15, C_TEXT, False),
    ("node_scan", "스캔", 15, C_TEXT, False),
    # [2026-09-16] 색 의미: 제목·비율은 흰색(중립), 배치는 초록, 낙하만 빨강.
    # 종전 '수확 완료' 는 C_ACCENT(0xFF6B81) 분홍빨강이라 에러처럼 읽혔다.
    ("final", "수확 완료", 30, C_TEXT, True),
    # [T4c 2026-09-15] 완료 둘째 줄 '배치 n · 낙하 m' (isaac_sim_viewport_display.HarvestHUD._row_final)
    ("final_placed", "배치", 20, C_OK, False),
    ("final_dropped", "낙하", 20, C_BAD, False),
    # [2026-09-16] 손목 카메라 창 제목 (isaac_sim_viewport_display._WristCamera) — 인식 결과가 아니라 렌더라는 것을 창에 적는다
    ("cam_title", "손목 카메라 · D455 컬러 렌더 · 인식 없음", 15, C_DIM, False),
] + [("state_" + k, v, 34, _rgb(PHASE[k]), True) for k, v in STATE_KO.items()] \
  + [("region_" + k, v, 24, C_TEXT, False) for k, v in REGION_KO.items()]


def render(text, size, color, bold):
    font = ImageFont.truetype(BOLD if bold else SRC, size * SCALE, index=FACE_KR)
    asc, desc = font.getmetrics()
    w = int(font.getlength(text)) + 2 * SCALE
    h = asc + desc
    img = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    ImageDraw.Draw(img).text((SCALE, 0), text, font=font, fill=color)
    return img


def render_parts(parts, size):
    """여러 색 글자를 한 장에 — '남서(분할)' 처럼 방향과 상태를 다른 색으로."""
    font = ImageFont.truetype(SRC, size * SCALE, index=FACE_KR)
    asc, desc = font.getmetrics()
    w = int(sum(font.getlength(t) for t, _ in parts)) + 2 * SCALE
    img = Image.new("RGBA", (w, asc + desc), (0, 0, 0, 0))
    draw, x = ImageDraw.Draw(img), SCALE
    for text, color in parts:
        draw.text((x, 0), text, font=font, fill=tuple(color))
        x += font.getlength(text)
    return img


if not os.path.exists(SRC):
    sys.exit("Noto CJK 가 없다: sudo apt install fonts-noto-cjk")
os.makedirs(OUT, exist_ok=True)
manifest = {}
for name, text, size, color, bold in ITEMS:
    img = render(text, size, color, bold)
    img.save(os.path.join(OUT, name + ".png"))
    manifest[name] = {"w": img.width / SCALE, "h": img.height / SCALE, "text": text}
for key in tree_model.TAG_KEYS:
    parts = tree_model.tag_parts(key)
    img = render_parts(parts, tree_model.TAG_SIZE)
    img.save(os.path.join(OUT, "tree_" + key + ".png"))
    manifest["tree_" + key] = {"w": img.width / SCALE, "h": img.height / SCALE,
                               "text": "".join(t for t, _ in parts)}
with open(os.path.join(OUT, "manifest.json"), "w", encoding="utf-8") as f:
    json.dump(manifest, f, ensure_ascii=False, indent=1)
print("wrote %d labels -> %s" % (len(manifest), OUT))
