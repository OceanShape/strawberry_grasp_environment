#!/bin/bash
cd /home/sun/strawberry_grasp_environment

# 1. ROS 2 환경 소싱 및 패키지 경로 설정
source /opt/ros/humble/setup.bash
export ROS_PACKAGE_PATH=/home/sun/strawberry_grasp_environment/src/doosan_robot2:/home/sun/strawberry_grasp_environment/src/RH-P12-RN:/opt/ros/humble/share:$ROS_PACKAGE_PATH

# 2. xacro 렌더링
echo "Generating robot.urdf from xacro..."
xacro src/strawberry_grasp_environment_description/urdf/robot.urdf.xacro > robot_export.urdf

# 3. Isaac Sim을 위해 package:// 경로를 절대 경로로 치환
echo "Replacing package:// paths with absolute paths for Isaac Sim..."
sed -i 's|package://dsr_description2|/home/sun/strawberry_grasp_environment/src/doosan_robot2/dsr_description2|g' robot.urdf
sed -i 's|package://rh_p12_rn_description|/home/sun/strawberry_grasp_environment/src/RH-P12-RN/rh_p12_rn_description|g' robot.urdf
sed -i 's|package://strawberry_grasp_environment_description|/home/sun/strawberry_grasp_environment/src/strawberry_grasp_environment_description|g' robot.urdf

echo "Done! The robot.urdf is ready for Isaac Sim."
