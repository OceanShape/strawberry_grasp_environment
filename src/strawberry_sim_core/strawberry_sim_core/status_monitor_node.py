"""파이프라인 상태 표시 창.

터미널 로그를 눈으로 좇지 않아도 로봇이 지금 무엇을 하는지 한눈에 보이게 한다.
항상 위(always-on-top) 작은 창을 화면 구석에 띄운다.

읽는 것만 한다 — 어떤 토픽도 발행하지 않고 서비스도 호출하지 않는다.
플래너 코드는 건드리지 않는다: 플래너의 세부 단계는 /rosout(모든 노드의 로그가
자동으로 흐르는 표준 토픽)을 구독해 문자열로 판별한다.

실행:
    ros2 run strawberry_sim_core status_monitor_node
"""

import queue
import threading
import time
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PoseArray
from rcl_interfaces.msg import Log

# ── 색 ────────────────────────────────────────────────────────────────────────
BG      = "#12151c"
FG      = "#e8edf5"
DIM     = "#7b8494"
PANEL   = "#1b202b"

C_IDLE  = "#7b8494"   # 대기
C_MOVE  = "#4a9eff"   # 이동
C_SCAN  = "#00c8c8"   # 탐지
C_PLAN  = "#a78bfa"   # 계획
C_APPR  = "#ffb547"   # 접근
C_GRASP = "#ff7a3d"   # 파지
C_PLACE = "#5ad469"   # 배치
C_DONE  = "#5ad469"   # 완료
C_WARN  = "#ffd34d"   # 경고
C_ERR   = "#ff4d5e"   # 오류

# (검사할 부분문자열, 표시 라벨, 색, 단계순서)  — 위에서부터 먼저 매칭
#   단계순서: 진행바 표시에만 사용. None이면 진행바를 건드리지 않는다.
RULES = [
    # 오류·거부 (최우선)
    ("START_REJECTED",           "거부됨",          C_ERR,   None),
    ("ABORT",                    "중단",            C_ERR,   None),
    ("failed/timeout",           "동작 실패",       C_ERR,   None),
    ("GRASP_EMPTY",              "파지 실패(빈손)", C_WARN,  6),
    # 완료·대기
    ("READY_FOR_NEXT_START",     "대기 (재시작 가능)", C_IDLE, 0),
    ("SCAN_COMPLETE",            "시퀀스 완료",     C_DONE,  9),
    ("AT_OVERVIEW",              "복귀 완료",       C_DONE,  8),
    ("RETURNING_TO_OVERVIEW",    "복귀 중",         C_MOVE,  8),
    # 픽 사이클
    # [FIX 2026-09-08] 순서·패턴 주의.
    # 플래너는 완료를 `=== PICK COMPLETE (DETACH_SUCCESS_UNVERIFIED) ===` 로 찍는다.
    # 종전 표는 언더바형 "PICK_COMPLETE" 만 있어 이 줄에 안 걸리고, 대신 부분문자열
    # "DETACH" 에 걸려 **픽이 끝났는데 "분리(당겨빼기)"로 표시**됐다.
    # 공백형을 먼저 넣고, DETACH 는 실제 분리 동작 로그만 잡도록 좁힌다.
    ("PICK COMPLETE",            "픽 완료",         C_DONE,  7),
    ("PICK_COMPLETE",            "픽 완료",         C_DONE,  7),
    ("pick_complete",            "픽 완료",         C_DONE,  7),
    ("PLACE",                    "배치 중",         C_PLACE, 7),
    ("DETACH_PULL_DOWN",         "분리(당겨빼기)",  C_GRASP, 7),
    ("detach pull",              "분리(당겨빼기)",  C_GRASP, 7),
    ("RETREAT",                  "후퇴",            C_MOVE,  7),
    ("retreat",                  "후퇴",            C_MOVE,  7),
    ("GRASP_CONTACT_DETECTED",   "파지 성공",       C_PLACE, 6),
    ("VERIFY_GRASP",             "파지 확인",       C_GRASP, 6),
    ("SAFE_GRASP",               "파지 중",         C_GRASP, 6),
    ("close gripper",            "그리퍼 닫는 중",  C_GRASP, 6),
    ("FINAL_APPROACH_STRAIGHT",  "직선 진입",       C_APPR,  5),
    ("PRE_APPROACH_REACHED",     "접근 지점 도달",  C_APPR,  4),
    ("PRE_APPROACH",             "접근 중",         C_APPR,  4),
    ("Plan OK",                  "계획 성공",       C_PLAN,  3),
    ("MoveSplineJoint",          "궤적 실행",       C_MOVE,  3),
    # 스캔
    ("TARGET_FOUND",             "타겟 확보",       C_SCAN,  2),
    ("AT_SCAN_POSE",             "탐지 중",         C_SCAN,  2),
    ("MOVING_TO",                "스캔 포즈 이동",  C_MOVE,  1),
    ("START_ACCEPTED",           "시작됨",          C_MOVE,  1),
]

PHASES = ["대기", "스캔이동", "탐지", "계획", "접근", "진입", "파지", "후퇴/배치", "복귀", "완료"]

WATCH_NODES = ("curobo_planner_node", "scan_executor_node", "sim_executor_bridge_node")


# 파이프라인을 구성하는 노드 (생존 표시용)
EXPECT_NODES = [
    ("fake_vision_node",         "vision"),
    ("sim_executor_bridge_node", "bridge"),
    ("curobo_planner_node",      "planner"),
    ("scan_executor_node",       "scan"),
]


class StatusMonitor(Node):
    def __init__(self, q: "queue.Queue"):
        super().__init__("status_monitor_node")
        self.q = q
        # /rosout 은 TRANSIENT_LOCAL 이라, 늦게 켜면 과거 로그가 한꺼번에 재생된다.
        # 창을 띄운 시점보다 오래된 메시지는 무시해서 단계 표시가 튀지 않게 한다.
        self.t0 = self.get_clock().now().nanoseconds * 1e-9 - 2.0
        self.create_subscription(String, "/strawberry/scan/status", self._status_cb, 10)
        # /rosout 은 TRANSIENT_LOCAL + BEST_EFFORT 조합으로 발행된다
        qos = QoSProfile(
            depth=100,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Log, "/rosout", self._log_cb, qos)
        # [2026-09-08] 딸기 수신 개수. 파이프라인 어디서 끊겼는지 한눈에 보려면
        # "Isaac 이 보낸 수" 와 "fake_vision 이 플래너로 넘긴 수" 를 나눠 봐야 한다.
        #   /isaac_sim/strawberries        : Isaac 브릿지 -> fake_vision (익은 것만 발행)
        #   /strawberry/detection/scene_positions : fake_vision -> 플래너 (x,y,z 평면 배열)
        # 둘이 다르면 ripeness 필터나 토픽 연결을 의심하면 된다.
        self.create_subscription(
            PoseArray, "/isaac_sim/strawberries", self._isaac_cb, 10)
        self.create_subscription(
            Float64MultiArray, "/strawberry/detection/scene_positions",
            self._scene_cb, 10)
        self.get_logger().info("Status monitor ready (읽기 전용)")

    def _isaac_cb(self, msg: PoseArray):
        self.q.put(("__isaac_count", str(len(msg.poses))))

    def _scene_cb(self, msg: Float64MultiArray):
        self.q.put(("__fake_count", str(len(msg.data) // 3)))

    def _status_cb(self, msg: String):
        self.q.put(("scan", msg.data))

    def _log_cb(self, msg: Log):
        if msg.name not in WATCH_NODES:
            return
        stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        if stamp < self.t0:
            return  # 창을 띄우기 전의 로그 (TRANSIENT_LOCAL 재생분)
        self.q.put((msg.name, msg.msg))

    def alive_nodes(self):
        try:
            return set(self.get_node_names())
        except Exception:
            return set()


class App:
    def __init__(self, q: "queue.Queue", node=None):
        self.q = q
        self.node = node
        self.phase_idx = 0
        self.t_start = None
        self.n_detect = 0
        self.n_pick = 0
        self.n_isaac = None      # Isaac -> fake_vision 수신 개수
        self.n_fake = None       # fake_vision -> 플래너 발행 개수
        # [2026-09-09] 분면 필터가 붙어 이 값은 **지금 스캔 중인 분면**의 개수다.
        # Isaac 쪽(전체 6개)과 다른 것이 정상이다. sw3/nw2/ne1/se0 이 설계값.
        self.clamp_warn = 0

        self.root = tk.Tk()
        self.root.title("딸기 수확 파이프라인")
        self.root.configure(bg=BG)
        self.root.attributes("-topmost", True)
        self.root.geometry("360x470+%d+40" % (self.root.winfo_screenwidth() - 380))

        f_big = tkfont.Font(family="DejaVu Sans", size=19, weight="bold")
        f_mid = tkfont.Font(family="DejaVu Sans", size=10)
        f_sml = tkfont.Font(family="DejaVu Sans Mono", size=8)

        tk.Label(self.root, text="ROBOT STATUS", bg=BG, fg=DIM,
                 font=tkfont.Font(family="DejaVu Sans", size=8, weight="bold")).pack(pady=(10, 2))

        self.lbl_phase = tk.Label(self.root, text="대기", bg=BG, fg=C_IDLE, font=f_big)
        self.lbl_phase.pack(pady=(0, 2))

        self.lbl_detail = tk.Label(self.root, text="트리거 대기 중", bg=BG, fg=DIM,
                                   font=f_mid, wraplength=330, justify="center")
        self.lbl_detail.pack(pady=(0, 8))

        # 진행 단계 표시
        self.cv = tk.Canvas(self.root, width=336, height=26, bg=BG, highlightthickness=0)
        self.cv.pack()
        self.cells = []
        w = 336 // len(PHASES)
        for i, name in enumerate(PHASES):
            r = self.cv.create_rectangle(i * w + 1, 2, (i + 1) * w - 1, 14,
                                         fill=PANEL, outline="")
            self.cv.create_text(i * w + w // 2, 21, text=name[:4], fill=DIM,
                                font=("DejaVu Sans", 5))
            self.cells.append(r)

        # 노드 생존 표시
        tk.Label(self.root, text="노드", bg=BG, fg=DIM,
                 font=("DejaVu Sans", 8)).pack(anchor="w", padx=12, pady=(8, 0))
        frn = tk.Frame(self.root, bg=BG); frn.pack(fill="x", padx=12)
        self.node_lbls = {}
        for full, short in EXPECT_NODES:
            l = tk.Label(frn, text=" %s " % short, bg=PANEL, fg=DIM,
                         font=("DejaVu Sans", 8))
            l.pack(side="left", padx=2)
            self.node_lbls[full] = l

        # 카운터
        fr = tk.Frame(self.root, bg=BG); fr.pack(pady=8, fill="x", padx=12)
        self.lbl_stat = tk.Label(fr, text="", bg=BG, fg=FG, font=f_sml, justify="left")
        self.lbl_stat.pack(anchor="w")

        tk.Label(self.root, text="최근 이벤트", bg=BG, fg=DIM,
                 font=("DejaVu Sans", 8)).pack(anchor="w", padx=12, pady=(4, 2))
        self.txt = tk.Text(self.root, height=11, bg=PANEL, fg=DIM, font=f_sml,
                           bd=0, highlightthickness=0, wrap="none")
        self.txt.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        self.root.after(100, self._tick)

    def _apply(self, src, text):
        if src == "__isaac_count":
            self.n_isaac = int(text); return
        if src == "__fake_count":
            self.n_fake = int(text); return
        # 이벤트 로그
        tag = {"scan": "SCAN", "curobo_planner_node": "PLAN",
               "sim_executor_bridge_node": "BRDG", "scan_executor_node": "SCAN"}.get(src, "----")
        self.txt.insert("1.0", "%s %s\n" % (tag, text[:60]))
        if int(self.txt.index("end-1c").split(".")[0]) > 60:
            self.txt.delete("60.0", "end")

        # 카운터
        if "clamped to 672mm" in text:
            self.clamp_warn += 1
        if "TARGET_FOUND" in text:
            try:
                self.n_detect = int([w for w in text.split() if w.isdigit()][0])
            except (IndexError, ValueError):
                pass
        if "PICK_COMPLETE" in text or "pick_complete" in text:
            self.n_pick += 1
        if "START_ACCEPTED" in text:
            self.t_start = time.time()
            self.n_pick = 0
            self.clamp_warn = 0

        # 단계 판정
        for key, label, color, idx in RULES:
            if key in text:
                self.lbl_phase.config(text=label, fg=color)
                self.lbl_detail.config(text=text[:110])
                if idx is not None:
                    self.phase_idx = idx
                    for i, r in enumerate(self.cells):
                        self.cv.itemconfig(r, fill=color if i <= idx else PANEL)
                break

    def _tick(self):
        try:
            while True:
                src, text = self.q.get_nowait()
                self._apply(src, text)
        except queue.Empty:
            pass

        # 노드 생존 갱신 (ROS 2 디스커버리 — 순서 무관, 켜지면 자동으로 잡힌다)
        if self.node is not None:
            alive = self.node.alive_nodes()
            for full, _short in EXPECT_NODES:
                on = full in alive
                self.node_lbls[full].config(fg=C_DONE if on else "#4a4f5c",
                                            bg=PANEL if on else BG)

        el = "--:--" if self.t_start is None else \
             time.strftime("%M:%S", time.gmtime(time.time() - self.t_start))
        clamp = ("정상 (0건)" if self.clamp_warn == 0
                 else "발생 %d건 !!" % self.clamp_warn)
        self.lbl_stat.config(
            text="경과 %s    탐지후보 %-4d 픽완료 %d\n"
                 "딸기 수신 : Isaac→fake %s개(전체)  fake→플래너 %s개(현재 분면)\n"
                 "M2 벽 clamp 경고 : %s"
                 % (el, self.n_detect, self.n_pick,
                    "--" if self.n_isaac is None else self.n_isaac,
                    "--" if self.n_fake is None else self.n_fake,
                    clamp))
        self.root.after(120, self._tick)


def main(args=None):
    q: "queue.Queue" = queue.Queue()
    rclpy.init(args=args)
    node = StatusMonitor(q)
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    app = App(q, node)
    try:
        app.root.mainloop()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
