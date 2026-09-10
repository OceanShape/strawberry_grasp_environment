# _baseline — 변경 추적 기준점

이 디렉토리는 **어떤 경우에도 수정하지 않는다** (`chmod a-w` 적용됨).
이 저장소는 git 커밋을 만들지 않으므로, 변경 이력은 커밋이 아니라
이 스냅샷과 `PLANNER_CHANGES.md` 대장으로 관리한다.

원칙: `PLANNER_POLICY_v2.md` / 로그: `PLANNER_CHANGES.md`

> **2026-09-06 v2 이후**: 이 스냅샷은 **삭제하지 않되 동기화 의무가 없다.**
> 유지 비용이 0이고, 나중에 "무엇을 어떻게 바꿨는지" 설명할 때 근거가 되기 때문에 남겨둔다.
검증: `scripts/check_planner.sh`

## A_strawberry_motion/ — 등급 A (실기 플래너)

**원본 대조본 확보됨.** 이식 과정에서 무엇을 바꿨는지 diff로 재생성할 수 있다.

- 출처: `~/motion_planning_node` (2026-07-08)
- 교차검증: `~/바탕화면/robotics_recent/motion_planning` (2026-07-02)와
  **바이트 단위 동일** — 독립 사본 2개가 일치하므로 원본으로 확정
- 대응 위치: `scripts/` → `src/strawberry_motion/scripts/`
             `execution/` → `src/strawberry_motion/strawberry_motion/execution/`

## B_e0509_gripper_description/ — 등급 B (설정 자산)

**원본 미확보.** baseline은 2026-09-05 현재 상태이며, 그 이전 변경분은 추적 불가다.
변경 내역은 `PLANNER_CHANGES.md`에 한 줄씩 남긴다.

- `logs/runtime/`은 실행 산출물이므로 스냅샷·diff 대상에서 제외
- ⚠️ cuRobo yml의 충돌 형상·조인트 한계, `environment.yaml`, 캘리브레이션 npz의
  수치를 바꿀 때는 `PLANNER_CHANGES.md`에 한 줄 남긴다 (나중에 설명할 수 있어야 한다).
