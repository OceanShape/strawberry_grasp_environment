import numpy as np
from scipy.spatial.transform import Rotation as R

# 1. 모션 플래너가 도출한 원본 캘리브레이션 행렬 (OpenCV 기준)
T_cv = np.array([
    [0.03299032, -0.42003111, 0.90690987, 0.06752496],
    [0.01449303, -0.90710734, -0.42064977, 0.07739811],
    [0.99935058,  0.02702124, -0.02383824, 0.01193509],
    [0.0,         0.0,         0.0,         1.0       ]
])

# 2. 좌표계 변환 행렬 생성 (OpenCV -> Isaac Sim)
# X축을 기준으로 180도 회전하여 축 기준을 일치시킴
R_cv_to_isaac = R.from_euler('x', 180, degrees=True).as_matrix()
T_cv_to_isaac = np.eye(4)
T_cv_to_isaac[:3, :3] = R_cv_to_isaac

# 3. 최종 행렬 곱셈 (원본 데이터를 Isaac Sim 기준으로 번역)
T_isaac = T_cv @ T_cv_to_isaac

# 4. Isaac Sim GUI에 입력할 Transform 값 추출
translation = T_isaac[:3, 3]
rotation_matrix = T_isaac[:3, :3]
euler_angles = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=True)

print("=== 🎯 Isaac Sim 완벽 적용 데이터 ===")
print(f"Translate (X, Y, Z): {translation[0]:.6f}, {translation[1]:.6f}, {translation[2]:.6f}")
print(f"Rotate (X, Y, Z): {euler_angles[0]:.6f}, {euler_angles[1]:.6f}, {euler_angles[2]:.6f}")
