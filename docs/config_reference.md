# 설정 파일 레퍼런스

> **담당: 설정 파일이 무엇이고 어느 노드가 읽는가.**
> **값 자체는 [`parameters.md`](parameters.md) 가 기준이다.**

이 프로젝트의 YAML/YML 설정 파일이 각각 무엇을 정의하고, 어떤 노드가 읽는지 정리합니다.

---

## 배치 구조: 소스 트리 vs share

```
src/strawberry_motion/config/          ← 소스 트리 (원본, 편집하는 곳)
src/e0509_gripper_description/config/  ← 〃 (데이터 전용 패키지)

    colcon build 시 ↓ 복사

install/<패키지>/share/<패키지>/config/  ← share (노드가 이름으로 찾는 설치본)
```

- **소스 트리**: `src/` 아래 원본. 설정을 수정할 땐 여기서.
- **share**: `colcon build`가 데이터 파일을 복사해 넣는 곳. 노드가 `get_package_share_directory("패키지명")`으로 찾는 위치. **수정 후에는 재빌드해야 share에 반영됩니다.**
- `e0509_gripper_description`은 코드 없이 설정만 담은 **데이터 전용 패키지**입니다. `sim_executor_bridge_node`가 이 패키지 이름으로 cuRobo 설정을 찾도록 코딩되어 있어서 만들었습니다.

---

## cuRobo 로봇 설정 (`config/curobo/`)

cuRobo는 GPU 가속 모션 플래닝 라이브러리입니다. "로봇이 어떻게 생겼고, 어디까지 움직일 수 있는지"를 이 파일들로 알려줘야 경로를 계산할 수 있습니다.

### `e0509_gripper.yml` — 로봇 정의서 (legacy 160mm 모델)

cuRobo에게 로봇의 구조를 알려주는 메인 설정. 주요 내용:

| 항목 | 의미 |
|---|---|
| `urdf_path` | 기구학 계산에 쓸 URDF 파일 (아래 `e0509_gripper.urdf`) |
| `base_link` / `ee_link` | 로봇의 시작 링크와 끝(엔드 이펙터) 링크. 여기선 `gripper_rh_p12_rn_base` |
| `collision_link_names` | 충돌 검사 대상 링크 목록 |
| `collision_spheres` | 충돌 근사 구체 파일 참조 (아래 `e0509_spheres.yml`) |
| `self_collision_ignore` | 인접 링크끼리는 충돌 검사 제외 (원래 붙어있으니까) |
| `attached_object` | 파지한 물체를 로봇 일부로 취급하기 위한 가상 링크 |

### `e0509_gripper_measured_tcp.yml` — 실측 TCP 모델 (⚠️ 검증 필요)

위와 같지만 `ee_link`가 그리퍼 베이스에서 +Z 100mm 떨어진 가상 링크(`grasp_tcp_link`)로 설정된 버전. 실측 플랜지→파지중심 거리(260mm)를 반영한 것인데, **실기팀이 이 100mm 오프셋을 확인해주기 전까지는 `legacy_160mm` 프로필 사용을 권장**합니다.

```bash
python3 curobo_planner_node.py --ros-args -p tool_model_profile:=legacy_160mm
```

### `e0509_spheres.yml` — 충돌 근사 구체

로봇의 실제 메시로 충돌 검사를 하면 너무 느려서, cuRobo는 **각 링크를 여러 개의 구(sphere)로 근사**해서 GPU에서 병렬로 검사합니다. 이 파일이 링크별 구체의 중심 좌표와 반지름을 정의합니다.

```yaml
link_1:
  - {"center": [0.0, 0.0, 0.0], "radius": 0.06}   # link_1 좌표계 기준
```

로봇이 장애물을 자꾸 스친다면 반지름을 키우고, 좁은 곳을 통과 못 하면 줄이는 식으로 튜닝합니다.

### `e0509_gripper.urdf` — cuRobo 전용 URDF

cuRobo가 기구학(FK/IK) 계산에 쓰는 단순화된 로봇 기술 파일. **Isaac Sim에 임포트하는 최상위 `robot.urdf`와는 별개**입니다 (그쪽은 시각화·물리용 풀 모델, 이쪽은 계산용 경량 모델).

### `environment.yaml` — 정적 장애물 정의

플래너의 충돌 월드에 추가할 고정 장애물 목록. 현재 내용은 **멀리 떨어진 더미 큐브 1개**뿐인데, 이는 의도된 우회책입니다: `curobo_planner_node`는 장애물이 하나도 없으면 기본 테이블을 Z=-0.02에 자동 추가하는데, 이게 legacy 프로필에서 `INVALID_START_STATE_WORLD_COLLISION` 오판을 일으켰습니다. 더미를 하나 넣어 그 자동 추가를 막은 것입니다.

---

## 스캔 동작 설정 (`src/strawberry_motion/config/`)

### `scan_pose_candidates_refit_candidate.yaml` — 스캔 포즈 정의

`scan_executor_node`가 사용하는 **작업 공간 스캔 위치 목록**:

| 항목 | 의미 |
|---|---|
| `curobo_start_joints_deg` | 스캔 시작 전 대기 자세 (관절 각도 6개, 도 단위) |
| `targets[].cell_id` | 스캔 구역 이름 (`root/nw`, `root/ne`, `root/se`, `root/sw`) |
| `targets[].endpoint_joints_deg` | 해당 구역을 바라보는 스캔 자세 |
| `use_for_automated_motion` | `true`여야 자동 모션 허용 (안전 게이트) |
| `collision_world_validated_for_motion` | 충돌 월드 검증 완료 플래그 (안전 게이트) |

⚠️ **현재 4개 구역이 전부 같은 관절 각도로 되어 있습니다** — 실기 환경에서 재조정(refit)하다 만 후보 파일이기 때문. 이 씬의 딸기 배치(`layout_layer.usd`)에 맞는 스캔 자세로 재설정이 필요합니다. J1/J2 swing 초과 문제(`docs/next_steps.md` Phase 5)의 해결 방안 2가 바로 이 파일 수정입니다.

### `scan_collision_world.yaml` — 스캔용 충돌 월드

`scan_executor_node`의 cuRobo 초기화에 쓰이는 정적 장애물(cuboid) 목록. **현재 비어 있습니다** (`objects: []`). 실기에선 화이트보드 큐보이드가 들어 있었으므로, 이 씬의 테이블·보드 위치에 맞는 큐보이드 추가 검토가 필요합니다.

---

## 어떤 노드가 어떤 파일을 읽나

| 노드 | 읽는 파일 | 탐색 방식 |
|---|---|---|
| `curobo_planner_node` (`planner_bootstrap`) | `e0509_gripper*.yml`, `e0509_spheres.yml`, `e0509_gripper.urdf`, `environment.yaml` | ① 소스 트리 `src/strawberry_motion/config/curobo/` → ② `e0509_gripper_description` share |
| `scan_executor_node` | 위 cuRobo 파일들 + `scan_pose_candidates_refit_candidate.yaml` + `scan_collision_world.yaml` | cuRobo 파일: `src/e0509_gripper_description/` 절대경로 / 스캔 YAML: `strawberry_motion` share |
| `sim_executor_bridge_node` | `e0509_gripper.yml`, `e0509_gripper.urdf` (MoveLine IK용) | `e0509_gripper_description` share |
