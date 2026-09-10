import omni.physx
from pxr import UsdPhysics, PhysxSchema, Usd, UsdGeom
import datetime
import os
import builtins

import glob

LOG_DIR = os.path.expanduser("~/strawberry_grasp_environment/log")
if not os.path.exists(LOG_DIR):
    os.makedirs(LOG_DIR)

# 새 로그 파일 이름 생성 로직 (예: collision_0621_000234.log)
date_str = datetime.datetime.now().strftime("%m%d")
existing_logs = glob.glob(os.path.join(LOG_DIR, f"collision_{date_str}_*.log"))
max_seq = -1
for log in existing_logs:
    try:
        seq_str = log.split("_")[-1].split(".")[0]
        max_seq = max(max_seq, int(seq_str))
    except ValueError:
        pass

next_seq = max_seq + 1
LOG_FILE = os.path.join(LOG_DIR, f"collision_{date_str}_{next_seq:06d}.log")

def _get_part_type(path: str) -> str:
    p = path.lower()
    if 'link6' in p: return 'link6'
    elif 'link5' in p: return 'link5'
    elif 'link4' in p: return 'link4'
    elif 'link3' in p: return 'link3'
    elif 'link2' in p: return 'link2'
    elif 'link1' in p: return 'link1'
    elif 'base' in p and 'rh' not in p: return 'base'
    elif 'rh' in p and 'base' in p: return 'rh_base'
    elif 'rh' in p and 'l1' in p: return 'rh_l1'
    elif 'rh' in p and 'l2' in p: return 'rh_l2'
    elif 'rh' in p and 'r1' in p: return 'rh_r1'
    elif 'rh' in p and 'r2' in p: return 'rh_r2'
    elif 'custom' in p and 'left' in p: return 'custom_part_left'
    elif 'custom' in p and 'right' in p: return 'custom_part_right'
    else: return 'unknown'

def _is_ignored_collision(path1: str, path2: str) -> bool:
    part1 = _get_part_type(path1)
    part2 = _get_part_type(path2)
    
    # 예외 케이스: 같은 파트끼리의 충돌 (자기 자신)
    if part1 == part2 and part1 != 'unknown':
        return True

    pair = frozenset([part1, part2])
    
    IGNORED_PAIRS = set([
        # 0. 로봇 바디 인접 링크 무시
        frozenset(['base', 'link1']),
        frozenset(['link1', 'link2']),
        frozenset(['link2', 'link3']),
        frozenset(['link3', 'link4']),
        frozenset(['link4', 'link5']),
        frozenset(['link5', 'link6']),
        
        # 1. rh base와 rh l1/r1 끼리의 충돌 무시
        frozenset(['rh_base', 'rh_l1']),
        frozenset(['rh_base', 'rh_r1']),
        
        # 2. rh l1과 rh l2, rh r1과 rh r2 무시
        frozenset(['rh_l1', 'rh_l2']),
        frozenset(['rh_r1', 'rh_r2']),
        
        # 3. rh l2와 custom_part_left, rh r2와 custom_part_right 무시
        frozenset(['rh_l2', 'custom_part_left']),
        frozenset(['rh_r2', 'custom_part_right']),
        
        # 4. 사용자가 제보한 추가 오탐지 무시 (link1/2와 엔드이펙터 계열)
        frozenset(['link1', 'link6']),
        frozenset(['link1', 'rh_base']),
        frozenset(['link1', 'rh_l1']),
        frozenset(['link1', 'rh_r1']),
        frozenset(['link1', 'rh_l2']),
        frozenset(['link1', 'rh_r2']),
        frozenset(['link1', 'custom_part_left']),
        frozenset(['link1', 'custom_part_right']),
        frozenset(['link2', 'custom_part_left']),
        frozenset(['link2', 'custom_part_right']),
    ])
    
    if pair in IGNORED_PAIRS:
        # print(f"Ignored: {part1} <---> {part2} (in IGNORED_PAIRS)")
        return True
        
    # 4. rh 계열 (및 커스텀) 부품들은 link6와의 충돌을 허용 (무시)
    rh_parts = ['rh_base', 'rh_l1', 'rh_l2', 'rh_r1', 'rh_r2', 'custom_part_left', 'custom_part_right']
    if 'link6' in pair:
        other_part = list(pair - set(['link6']))
        if other_part and other_part[0] in rh_parts:
            # print(f"Ignored: {part1} <---> {part2} (link6 with rh_parts)")
            return True

    # print(f"Not ignored: {part1} <---> {part2}")
    return False

def _get_path_from_handle(handle):
    try:
        # 1. PhysicsSchemaTools.intToSdfPath 시도
        from pxr import PhysicsSchemaTools
        path = PhysicsSchemaTools.intToSdfPath(int(handle))
        if path and str(path) != "":
            return str(path)
    except Exception:
        pass
    
    try:
        # 2. omni.physx interface 시도
        import omni.physx
        physx_sim_iface = omni.physx.get_physx_simulation_interface()
        if hasattr(physx_sim_iface, "get_collider_path"):
            return str(physx_sim_iface.get_collider_path(int(handle)))
    except Exception:
        pass
        
    return str(handle) # 둘 다 실패하면 원래 숫자 반환

def on_contact_report_event(contact_headers, contact_data):
    import omni.timeline
    if not omni.timeline.get_timeline_interface().is_playing():
        return

    # Joint 상태 문자열 포맷팅
    j_str = "unknown_joints"
    try:
        if hasattr(builtins, "my_robot") and builtins.my_robot is not None:
            pos = builtins.my_robot.get_joint_positions()
            names = builtins.my_robot.dof_names
            
            if pos is not None and names is not None:
                # 포맷 매핑용 이름 축약
                short_names = []
                for n in names:
                    if '1' in n and 'joint' in n: short_names.append("j1")
                    elif '2' in n and 'joint' in n: short_names.append("j2")
                    elif '3' in n and 'joint' in n: short_names.append("j3")
                    elif '4' in n and 'joint' in n: short_names.append("j4")
                    elif '5' in n and 'joint' in n: short_names.append("j5")
                    elif '6' in n and 'joint' in n: short_names.append("j6")
                    elif 'l1' in n: short_names.append("rh_l1")
                    elif 'r1' in n: short_names.append("rh_r1")
                    elif 'l2' in n: short_names.append("rh_l2")
                    elif 'r2' in n: short_names.append("rh_r2")
                    elif 'base' in n and 'rh' in n: short_names.append("rh_base")
                    else: short_names.append(n)
                
                j_str = "/".join([f"{sn}:{float(p):.2f}" for sn, p in zip(short_names, pos)])
            else:
                j_str = "sim_stopped"
        else:
            j_str = "no_bridge_robot"
    except Exception as e:
        j_str = f"joint_err: {str(e)[:15]}"

    for contact_header in contact_headers:
        try:
            # actor0, actor1이 ID(정수)인 경우 실제 경로(문자열)로 변환
            actor0_path = _get_path_from_handle(contact_header.actor0)
            actor1_path = _get_path_from_handle(contact_header.actor1)
            
            # 둘 다 로봇 내부 부품인지 확인
            is_robot0 = "robot" in actor0_path.lower()
            is_robot1 = "robot" in actor1_path.lower()
            
            part0 = _get_part_type(actor0_path)
            part1 = _get_part_type(actor1_path)
            
            # 파츠 이름이 unknown이면 그냥 원래 경로의 마지막 이름 사용
            if part0 == "unknown": part0 = actor0_path.split("/")[-1]
            if part1 == "unknown": part1 = actor1_path.split("/")[-1]
            
            if is_robot0 and is_robot1:
                is_ignored = _is_ignored_collision(actor0_path, actor1_path)
                status = "[IGNORED] SELF-COLLISION" if is_ignored else "SELF-COLLISION"
            else:
                # 로봇끼리의 충돌이 아닌 경우 (환경 오브젝트 등)
                if "ground" in actor0_path.lower() or "ground" in actor1_path.lower():
                    continue # 바닥 충돌은 스팸 방지
                status = "ENV-COLLISION"
                
            # [시간] (관절상태) 부품 <---> 부품 형식으로 출력
            time_str = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
            
            # 필터링된 자기 충돌은 무시 (로그에 안 남김) - 원하시면 아래 주석을 풀고 남길 수 있음
            if is_ignored:
                # msg = f"[{time_str}] ({j_str}) {part0} <---> {part1} [IGNORED]\n"
                continue
            else:
                msg = f"[{time_str}] ({j_str}) {part0} <---> {part1}\n"
            
            print(msg.strip())
            with open(LOG_FILE, "a") as f:
                f.write(msg)
                    
        except Exception as e:
            print(f"Error in contact report: {e}")

def start_collision_logger():
    # 1. 이전 구독이 있다면 해제
    if hasattr(builtins, "my_collision_sub") and builtins.my_collision_sub is not None:
        builtins.my_collision_sub = None
        
    stage = omni.usd.get_context().get_stage()
    if not stage:
        print("No stage loaded.")
        return

    # 2. 로봇의 모든 충돌체에 ContactReportAPI 적용 및 Threshold 설정
    applied_count = 0
    articulation_enabled = False
    
    for prim in stage.Traverse():
        # Articulation에 자기 충돌(Self-Collision)이 꺼져있으면 아예 물리엔진이 무시하므로 강제로 켬
        if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
            if "robot" in str(prim.GetPath()).lower():
                art_api = PhysxSchema.PhysxArticulationAPI.Get(stage, prim.GetPath())
                if art_api:
                    art_api.CreateEnabledSelfCollisionsAttr().Set(True)
                    articulation_enabled = True
                    
        # 모든 껍데기(도형)에 대해 물리 충돌 활성화(CollisionEnabled) 강제 적용
        if "robot" in str(prim.GetPath()).lower():
            
            # 만약 사용자가 만든 Sphere, Capsule 등의 기본 도형에 CollisionAPI가 안 달려 있다면 강제로 달아줌
            if prim.IsA(UsdGeom.Capsule) or prim.IsA(UsdGeom.Sphere) or prim.IsA(UsdGeom.Cube) or prim.IsA(UsdGeom.Cylinder):
                if not prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI.Apply(prim)
                    
            # 만약 Mesh(또는 외부 임포트 큐브 등)인데 이름이나 상위 폴더에 'collision'이 들어있다면 강제로 물리 속성 부여
            if prim.IsA(UsdGeom.Mesh) and "collision" in str(prim.GetPath()).lower():
                if not prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI.Apply(prim)
                if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                    UsdPhysics.MeshCollisionAPI.Apply(prim)
                
                # 강체(Dynamic)는 무조건 Convex Hull이어야 물리 연산(초록색 선)이 활성화됨
                mesh_api = UsdPhysics.MeshCollisionAPI.Get(stage, prim.GetPath())
                if mesh_api:
                    mesh_api.CreateApproximationAttr().Set("convexHull")
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                col_api = UsdPhysics.CollisionAPI.Get(stage, prim.GetPath())
                col_api.CreateCollisionEnabledAttr().Set(True)
            if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                pass # MeshCollisionAPI doesn't have collisionEnabled, it's on CollisionAPI
                
        # 물리 충돌 리포트는 RigidBodyAPI가 있는 프림(링크)에 달아야 정상 동작하며, simulationOwner 경고를 방지합니다.
        has_rigid_body = prim.HasAPI(UsdPhysics.RigidBodyAPI)
        
        # Check if the prim is inside the robot path, and has RigidBodyAPI
        if "robot" in str(prim.GetPath()).lower() and has_rigid_body:
            path_str = str(prim.GetPath())
            if "robot" in path_str.lower():
                report_api = PhysxSchema.PhysxContactReportAPI.Get(stage, prim.GetPath())
                if not report_api:
                    report_api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
                
                # 기본 임계값(Threshold)을 그대로 사용하도록 설정 제거 (가끔 0.0이 무시되는 버그 방지)
                # report_api.CreateThresholdAttr().Set(0.0)
                applied_count += 1
                
    # 스테이지 내의 모든 Collision Group 찾아서 확인 (유령처럼 통과하는 원인 파악용)
    collision_groups = []
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.CollisionGroup):
            collision_groups.append(str(prim.GetPath()))
            
    print(f"[Collision Logger] Found Collision Groups in Stage: {collision_groups}")
    print(f"[Collision Logger] Enabled Articulation Self-Collision: {articulation_enabled}")
    print(f"[Collision Logger] Applied PhysxContactReportAPI to {applied_count} robot colliders.")
    
    # 3. Contact Report 콜백 구독
    physx_sim_iface = omni.physx.get_physx_simulation_interface()
    builtins.my_collision_sub = physx_sim_iface.subscribe_contact_report_events(on_contact_report_event)
    
    # 4. 로그 파일 초기화 메세지
    with open(LOG_FILE, "a") as f:
        f.write(f"\n--- Logger Started at {datetime.datetime.now()} ---\n")
        f.write(f"Applied PhysxContactReportAPI to {applied_count} robot colliders.\n")
        
    print(f"[Collision Logger] Started successfully! Applied to {applied_count}. Logs will be saved to: {LOG_FILE}")

start_collision_logger()
