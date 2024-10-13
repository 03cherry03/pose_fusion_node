from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pose_fusion',  # 패키지 이름
            executable='pose_fusion_node',  # 실행 파일 이름
            name='pose_twist_fusion_node',
            output='screen',
            parameters=[
                {'lidar_pose_topic': '/localization/pose_twist_fusion_filter/kinematic_state_lidar'},
                {'gnss_pose_topic': '/localization/pose_twist_fusion_filter/kinematic_state_gnss'},
                {'can_topic': '/can_odom'},
                {'velocity_threshold': 0.5},
                {'angular_threshold': 0.05},
                {'lidar_pose_weight': 0.5},
                {'gnss_pose_weight': 0.5},
                {'lidar_twist_weight': 0.5},
                {'gnss_twist_weight': 0.5}
            ]
        )
    ])
