#!/bin/bash
PROJECT=/home/oceanshape/strawberry_grasp_environment
cd "$PROJECT"

# 1. ROS 2 환경 소싱 및 패키지 경로 설정
source /opt/ros/humble/setup.bash
export ROS_PACKAGE_PATH=$PROJECT/src/doosan_robot2:$PROJECT/src/RH-P12-RN:$PROJECT/src:/opt/ros/humble/share:$ROS_PACKAGE_PATH

# 2. xacro 렌더링
echo "Generating robot.urdf from xacro..."
xacro src/strawberry_grasp_environment_description/urdf/robot.urdf.xacro > robot.urdf

# 3. Isaac Sim을 위해 package:// 경로를 절대 경로로 치환
echo "Replacing package:// paths with absolute paths for Isaac Sim..."
sed -i "s|package://dsr_description2|$PROJECT/src/doosan_robot2/dsr_description2|g" robot.urdf
sed -i "s|package://rh_p12_rn_description|$PROJECT/src/RH-P12-RN/rh_p12_rn_description|g" robot.urdf
sed -i "s|package://strawberry_grasp_environment_description|$PROJECT/src/strawberry_grasp_environment_description|g" robot.urdf

# 4. RH-P12-RN 색상 커스터마이제이션 (원본 grey → black)
echo "Applying color customizations..."
sed -i 's|<material name="grey"/>|<material name="black"/>|g' robot.urdf
sed -i 's|Gazebo/Grey|Gazebo/FlatBlack|g' robot.urdf

echo "Done! robot.urdf is ready for Isaac Sim."
