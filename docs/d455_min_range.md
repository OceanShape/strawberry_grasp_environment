# D455 최소 측정 거리 vs 깊이 2 스캔 자세 — 실기 카메라 로그 기반 (2026-09-15)

**질문**: 깊이 2 세부 자세(실기 티칭 평면 ee y 0.433, 카메라-보드 318~319mm)에서 D455 가 깊이를 잴 수 있는가.

**1차 출처**: 실기 카메라의 `rs-enumerate-devices -c` 출력 — 사용자가 실기→시뮬 이관 때 받아 둔 로그
[`lab_data/realsense_d455_enumerate.txt`](lab_data/realsense_d455_enumerate.txt)
(D455, FW 5.17.0.10, Advanced Mode YES). 이 파일에는 **장치 제원(스트림 목록·내부/외부 파라미터)만 있고,
실제로 쓴 스트림 해상도·disparity shift·depth preset 은 없다** → 실기 설정은 **미확인**으로 둔다.

## 산식 (인텔 공식)

인텔 D400 튜닝 문서(BKM): **MinZ(mm) = 깊이 초점거리 Fx(px) × 베이스라인(mm) / 126** (126 = 스테레오 매칭의
disparity 탐색 폭). MinZ 는 가로 해상도에 비례해 줄고, disparity shift 를 올리면 MaxZ 를 희생하고 더 줄일 수 있다.
인텔 제품 페이지는 D455 운용 범위를 0.6~6m 로, 지원센터는 기본 Min-Z 를 0.4m 로 적는다(둘 다 고해상도 기준).

- 출처: [Intel D455 specifications](https://www.intel.com/content/www/us/en/products/sku/205847/intel-realsense-depth-camera-d455/specifications.html) ·
  [BKMs for Tuning D4xx (PDF)](https://www.realsenseai.com/wp-content/uploads/2019/11/BKMs_Tuning_RealSense_D4xx_Cam.pdf) ·
  [Tuning depth cameras for best performance](https://dev.intelrealsense.com/docs/tuning-depth-cameras-for-best-performance)

## 로그에서 읽은 값 → 해상도별 MinZ (disparity shift 0)

베이스라인 = "Infrared 1" → "Infrared 2" 외부 파라미터 이동 x = **95.03 mm** (로그 `Translation Vector: -0.0950324758887291`).

| 깊이 스트림 | Fx (px, 로그) | 깊이 FOV (로그) | MinZ = Fx×95.03/126 |
|---|---|---|---|
| 1280×720 | 657.0 | 88.5°×57.4° | **496 mm** |
| 848×480 (인텔 권장) | 435.3 | 88.5°×57.7° | **328 mm** |
| 640×480 | 394.2 | 78.1°×62.7° | **297 mm** |
| 640×360 | 328.5 | 88.5°×57.4° | **248 mm** |
| 480×270 | 246.4 | 88.5°×57.4° | 186 mm |
| 424×240 | 217.6 | 88.5°×57.7° | 164 mm |

(1280×720 의 496mm 는 인텔이 말하는 "약 0.5m" 와 맞는다 — 산식·베이스라인이 이 개체와 일치한다는 확인.)

## 우리 스캔 자세와의 대조

카메라 광학 중심 = USD 오프셋(툴 축 앞 63mm, `subcell_pose.CAMERA_OFFSET_IN_EE_M`). 보드 y=810, 과실 중심 y≈783(보드 앞 27mm),
과실 앞면은 그보다 약 22mm 더 앞.

| 자세 | 카메라-보드 | 카메라-과실 중심 | 카메라-과실 앞면 |
|---|---|---|---|
| 깊이 1 (분면, 4자세) | 412~472 mm | 385~445 mm | 363~423 mm |
| 깊이 2 (lab_plane, y 0.433) | 318~319 mm | ≈292 mm | ≈270 mm |

- **1280×720**: 깊이 1 도 MinZ(496) 아래 — 실기가 이 해상도로 분면 스캔을 했을 리 없다(또는 disparity shift 를 썼다).
- **848×480**: 깊이 1 은 안(385 > 328), **깊이 2 는 밖**(292 < 328).
- **640×480 이하**: 깊이 2 과실 중심이 경계 근처(297) \~ 안(248).
- **거리를 재는 원점**: 위 표의 카메라-보드 거리는 `subcell_pose.CAMERA_OFFSET_IN_EE_M` 기준이고, 그 값은 USD 의 `rsd455` 모듈 원점이다. 09-16 에 `check_wrist_camera_projection.py` 로 대조하니 모듈 원점은 `Camera_Pseudo_Depth` 원점과 같았다 — MinZ 를 따지는 프레임으로 맞다. 컬러 카메라 광학 원점은 ee 프레임에서 11.5mm 떨어져 있지만 분면 4자세에서 보드 거리 차이는 0.3mm 이하라 위 판정은 바뀌지 않는다(분면 411.7\~472.3mm, `log/m3/offline_checks/wrist_camera_projection_20260916.txt`).

→ 실기가 깊이 2 티칭 평면에서 실제로 인식했다면 640×480 이하이거나 disparity shift > 0 이었어야 한다. **그 설정을 적은 로그가 없으므로
단정하지 않는다.** 실기 4개 티칭 자세가 y=433 한 평면에 있다는 사실(G §5)까지만 말한다.

## 시뮬에 반영하지 않는 이유 (규칙 0)

`fake_vision` 은 카메라 거리 모델이 없다 — 이 축에서는 **시뮬이 원본보다 관대**하다. 실기 설정을 모르는 채 MinZ 를 골라 넣으면
근거 없는 값이 시뮬 동작을 정한다(설정을 알게 되면 `fake_vision` 에 최소 거리 하나로 넣을 수 있다). 지금은 **한계로 기록**한다
(H §9 유형). 면접 금지 표현: "실기가 깊이 2 자세에서 봤다" (G §9).
