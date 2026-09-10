"""덩굴 곡선 기하 — 프로토타입 1개(딸기 로컬 프레임). 생성기와 검증기가 공용."""
import numpy as np

# 로컬 프레임: 원점 = 과실 중심, 축은 world 와 동일 (딸기 orient 는 항등, scale 은 별도)
Y_BOARD = 0.0292     # 보드면(world y=0.810) 뒤 2mm — 끝단 캡을 보드로 가린다
Z_BOARD = 0.082      # 보드 부착점 (과실 중심 위)
Z_BEND = 0.065       # 여기서부터 수직 — 파지점(+35)·하강 시점(+65) 을 모두 수직 구간에 둔다
Z_TIP = 0.0250       # 덩굴 끝 — 메시 줄기(+26.4~+29.5mm) 를 지나 꽃받침까지 4.5mm 파묻는다
TIP = np.array([0.0, 0.0, Z_TIP])
R_BOARD, R_TIP = 0.0020, 0.0016             # 반지름 4.0 -> 3.2mm 지름
D1, D2 = 0.019, 0.012                       # 베지어 접선 길이

A = np.array([0.0, Y_BOARD, Z_BOARD])
C1 = A + np.array([0.0, -D1, 0.0])
B = np.array([0.0, 0.0, Z_BEND])
C2 = B + np.array([0.0, 0.0, D2])


def centerline(n_bend=14, n_vert=6):
    t = np.linspace(0.0, 1.0, n_bend)[:, None]
    bez = ((1 - t) ** 3 * A + 3 * (1 - t) ** 2 * t * C1
           + 3 * (1 - t) * t ** 2 * C2 + t ** 3 * B)
    s = np.linspace(0.0, 1.0, n_vert + 1)[1:, None]
    return np.vstack([bez, B + s * (TIP - B)])


def radii(pts):
    d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    return R_BOARD + (R_TIP - R_BOARD) * (d / d[-1])


def tube(pts, rad, sides=10):
    """중심선 -> 삼각형 없는 사각 격자 튜브 (points, faceVertexCounts, faceVertexIndices)."""
    tang = np.zeros_like(pts)
    tang[1:-1] = pts[2:] - pts[:-2]
    tang[0] = pts[1] - pts[0]
    tang[-1] = pts[-1] - pts[-2]
    tang /= np.linalg.norm(tang, axis=1)[:, None]
    ref = np.array([1.0, 0.0, 0.0])          # 곡선이 yz 평면 안에 있으므로 x 가 항상 유효
    verts, ang = [], np.linspace(0, 2 * np.pi, sides, endpoint=False)
    for p, t, r in zip(pts, tang, rad):
        u = ref - np.dot(ref, t) * t
        u /= np.linalg.norm(u)
        v = np.cross(t, u)
        verts.append(p + r * (np.cos(ang)[:, None] * u + np.sin(ang)[:, None] * v))
    verts = np.vstack(verts)
    counts, idx = [], []
    n = len(pts)
    for i in range(n - 1):
        for j in range(sides):
            k = (j + 1) % sides
            counts.append(4)
            idx += [i * sides + j, i * sides + k, (i + 1) * sides + k, (i + 1) * sides + j]
    # 끝단 캡 (보드 쪽만; 줄기 끝은 메시 안에 묻힌다)
    counts.append(sides)
    idx += [j for j in range(sides - 1, -1, -1)]
    return verts, np.array(counts), np.array(idx)
