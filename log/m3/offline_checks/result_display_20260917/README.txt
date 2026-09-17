2026-09-17 표시·용어 정리 검증 자료. 설명·결론은 docs/result_display_audit.md §7 이 1차 출처다.

- investigation_A_to_D_agents.txt : 로그 조사 A~D 에이전트 12개 보고 원문(배치 실패 4건 추적·런 식별표·반박 검증 / 분리·배치 실패 신호 조사 2건·반박 검증 / '낙하' 표기 전수 3건·완전성 검수 / 머티리얼 조사·반박 검증). JSON 구조 그대로.
- material_override_agents.txt    : 안 익은 딸기 색 오버라이드 구현 보고 + 반박 검증 2건(합성 정확성 / 금지·회귀). must_fix 0.
- hud_result_bar_agents.txt       : HUD 결과 바·타겟 수 검증 보고 2건 — Kit 스텁 하네스(isaac_sim_viewport_display.py 실행, 4조합 x 85검사), 프로브 재생(실제 PickSequenceExecutor.run 에 17:15 런 순서). **수정 전 코드 기준**이다.
    이 보고의 note 중 6건(bus_merge run 비 dict 퇴행, 빈손 판정 픽의 분리 실패 칸, append 실패 로그, HUD 로그 None, 씬 집계 로그 두 줄, check_camera_framing HUD 영역)은 같은 날 고쳤다.
    수정 후 같은 하네스를 다시 돌린 결과: 스텁 하네스 조합마다 84 통과 + 1(None 결함을 확인하던 정보성 검사가 결함 수정으로 뒤집힘),
    프로브 재생 T1~T3·T6~T9 통과, T4 는 한 항목만 뒤집힘(파지 판정 없이 분리 함수만 호출해도 칸이 붙는다는 수정 전 기대 — 파지 판정 CONTACT 픽만 세도록 바꾼 결과).
    하네스 스크립트와 수정 후 재실행 출력 파일은 세션 스크래치에 있었고 남지 않았다(재실행 결과는 위 문장과 docs 문서에 적은 것이 전부).
- gen_unripe_appearance_run.txt   : 생성기 실행 출력(종료코드 0, 저장 뒤 재오픈 검증 OK). 다시 돌려도 appearance_layer.usd md5 aa8da9db… 동일.
- check_params.txt                : python3 check_params.py 출력(종료코드 0, 전부 정합).
