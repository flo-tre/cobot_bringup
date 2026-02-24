from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = {"use_sim_time": True}

    joint_controllers_file = os.path.join(
        get_package_share_directory('cobot_bringup'), 'config', 'cobot_controllers_gripper.yaml'
    )
    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )

    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name="cobot_camera_gripper_moveit_config")
        .robot_description(
            file_path="config/cobot.urdf.xacro",
            mappings={
                "sim_gazebo": "true",
                "export_mimic_joint_interfaces": "true",
            },
        )
        .robot_description_semantic(file_path="config/cobot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True, publish_planning_scene=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    x_arg = DeclareLaunchArgument('x', default_value='0', description='X position of the robot')
    y_arg = DeclareLaunchArgument('y', default_value='0', description='Y position of the robot')
    z_arg = DeclareLaunchArgument('z', default_value='0', description='Z position of the robot')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'debug': 'false',
            'gui': 'true',
            'paused': 'false',
            #'world' : world_file
        }.items()
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("cobot_camera_gripper_moveit_config"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            use_sim_time,
        ],
    )

    # spawn the robot (with increased timeout)
    spawn_the_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'cobot',
            '-topic', 'robot_description',
            '-timeout', '120',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z')
        ],
        output='screen',
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[moveit_config.robot_description, use_sim_time],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[use_sim_time],
    )

    arm_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[use_sim_time],
    )

    gripper_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[use_sim_time],
    )

    config_dict = moveit_config.to_dict()
    config_dict.update(use_sim_time)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict, use_sim_time],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Sequence: spawn robot -> joint_state_broadcaster -> arm_trajectory_controller
    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_the_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_trajectory_controller_spawner, gripper_position_controller_spawner],
        )
    )

    delay_rviz_node = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[rviz_node],
        )
    )

    # Launch Description
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_the_robot)
    ld.add_action(delay_joint_state_broadcaster)
    ld.add_action(delay_arm_controller)
    ld.add_action(move_group_node)
    ld.add_action(delay_rviz_node)

    return ld