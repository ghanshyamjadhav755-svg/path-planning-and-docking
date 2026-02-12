import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'my_bot'

    # ----------------------------------------
    # Robot State Publisher
    # ----------------------------------------
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # ----------------------------------------
    # Gazebo
    # ----------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    # ----------------------------------------
    # Spawn Robot
    # ----------------------------------------
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_bot'
        ],
        output='screen'
    )

    # ----------------------------------------
    # RViz
    # ----------------------------------------
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'view_bot.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # ----------------------------------------
    # AprilTag Node
    # ----------------------------------------
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        parameters=[
            os.path.expanduser(
                '~/dev_ws/src/apriltag_ros/cfg/tags_36h11.yaml'
            )
        ],
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info')
        ]
    )

    # ----------------------------------------
    # Docking Script (Standalone Python)
    # ----------------------------------------
    tag_align = ExecuteProcess(
        cmd=[
            'python3',
            '/home/jd/dev_ws/src/my_bot/scripts/docking.py'
        ],
        output='screen'
    )
    docking_nav2 = ExecuteProcess(
        cmd=[
            'python3',
            '/home/jd/dev_ws/src/my_bot/scripts/docking_nav2.py'
        ],
        output='screen'
    )

    # ----------------------------------------
    # Launch Description
    # ----------------------------------------
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        rviz_node,
        apriltag_node,
        tag_align,
        docking_nav2
    ])



# import os
# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node


# def generate_launch_description():

#     package_name = 'my_bot'

#     # ----------------------------------------
#     # Robot State Publisher
#     # ----------------------------------------
#     rsp = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             os.path.join(
#                 get_package_share_directory(package_name),
#                 'launch',
#                 'rsp.launch.py'
#             )
#         ]),
#         launch_arguments={'use_sim_time': 'true'}.items()
#     )

#     # ----------------------------------------
#     # Gazebo Server + Client
#     # ----------------------------------------
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             os.path.join(
#                 get_package_share_directory('gazebo_ros'),
#                 'launch',
#                 'gazebo.launch.py'
#             )
#         ])
#     )

#     # ----------------------------------------
#     # Spawn Robot Entity
#     # ----------------------------------------
#     spawn_entity = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=[
#             '-topic', 'robot_description',
#             '-entity', 'my_bot'
#         ],
#         output='screen'
#     )

#     # ----------------------------------------
#     # RViz2 with Config File
#     # ----------------------------------------
#     rviz_config_path = os.path.join(
#         get_package_share_directory(package_name),
#         'config',
#         'view_bot.rviz'
#     )

#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         arguments=['-d', rviz_config_path],
#         output='screen'
#     )

#     # ----------------------------------------
#     # Launch Description
#     # ----------------------------------------
#     return LaunchDescription([
#         rsp,
#         gazebo,
#         spawn_entity,
#         rviz_node,
#     ])

