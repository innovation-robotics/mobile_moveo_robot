import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro
import argparse
import sys
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


# Mapping from entry in the robot_body configuration to joints in the SRDF
# CONFIGURATION_TRANSLATION = {
#     'lift': ['lift'],
#     'wrist_yaw': ['wrist_yaw'],
#     'robot_gripper': ['joint_gripper_finger_left', 'joint_gripper_finger_right'],
#     'base': ['position/x', 'position/theta'],
#     'arm': ['joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3'],
#     'head': ['joint_head_pan', 'joint_head_tilt']
# }

def load_joint_limits_from_config(mode='default'):
    """Translate the values from the robot configuration to params to be used by MoveIt."""
    print('Load from default')
    return load_yaml('moveit_resources_robot_moveit_config', 'config/joint_limits.yaml')


def generate_launch_description():
    package_name='moveit_resources_robot_moveit_config' #<--- CHANGE ME
    parser = argparse.ArgumentParser()
    parser.add_argument("--use_fake_controller", default=False, type=eval, choices=[True, False])
    args, _ = parser.parse_known_args([arg for sys_arg in sys.argv[4:] for arg in ('--' + sys_arg).split(':=')])

    ld = LaunchDescription()
    # planning_context
    robot_description_config = xacro.process_file(os.path.join(get_package_share_directory('moveit_resources_robot_moveit_config'),
                                                            'config',
                                                            'robot.urdf.xacro'),
                                                            mappings={'use_fake_controller': str(args.use_fake_controller)})
    robot_description = {'robot_description' : robot_description_config.toxml()}

    robot_description_semantic_config = load_file('moveit_resources_robot_moveit_config', 'config/robot.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    kinematics_yaml = load_yaml('moveit_resources_robot_moveit_config', 'config/kinematics.yaml')

    joint_limits_yaml = {'robot_description_planning': load_joint_limits_from_config()}
    # joint_limits_yaml = load_joint_limits_from_config()

    # Planning Functionality
    ompl_planning_pipeline_config = { 'move_group' : {
        'planning_plugin' : 'ompl_interface/OMPLPlanner',
        'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
        'start_state_max_bounds_error' : 0.1 } }
    ompl_planning_yaml = load_yaml('moveit_resources_robot_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # # Trajectory Execution Functionality
    controllers_yaml = load_yaml('moveit_resources_robot_moveit_config', 'config/moveit_controllers.yaml')
    # controllers_yaml = load_yaml('moveit_resources_robot_moveit_config', 'config/ros_controllers.yaml')
    moveit_controllers = { 'moveit_simple_controller_manager' : controllers_yaml,
                            'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}

    trajectory_execution = {'allow_trajectory_execution': True,
                            'moveit_manage_controllers': False,
                            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                            'trajectory_execution.allowed_goal_duration_margin': 0.5,
                            'trajectory_execution.allowed_start_tolerance': 3.0}

    planning_scene_monitor_parameters = {"publish_planning_scene": True,
                                        "publish_geometry_updates": True,
                                        "publish_state_updates": True,
                                        "publish_transforms_updates": True}

    # Start the actual move_group node/action server
    run_move_group_node = Node(package='moveit_ros_move_group',
                            executable='move_group',
                            output='screen',
                            parameters=[robot_description,
                                        robot_description_semantic,
                                        kinematics_yaml,
                                        joint_limits_yaml,
                                        ompl_planning_pipeline_config,
                                        trajectory_execution,
                                        moveit_controllers,
                                        planning_scene_monitor_parameters,
                                        {"use_sim_time": True}
                                        # sensors_3d_yaml
                                        ])
    ld.add_action(run_move_group_node)

    # RViz
    rviz_config_file = get_package_share_directory('moveit_resources_robot_moveit_config') + "/launch/moveit_with_nav_ignition_grouped.rviz"
    rviz_node = Node(package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='log',
                    arguments=['-d', rviz_config_file],
                    parameters=[robot_description,
                                robot_description_semantic,
                                ompl_planning_pipeline_config,
                                kinematics_yaml,
                                {"use_sim_time": True}])
    # rviz_node = Node(package='rviz2',
    #                 executable='rviz2',
    #                 name='rviz2',
    #                 output='log',
    #                 parameters=[robot_description])
    ld.add_action(rviz_node)

    # Static TF
    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "odom", "map"],
    # )
    # ld.add_action(static_tf)

    # # # Static TF Laser
    # static_laser_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["0.437", "0.255", "0.0", "3.141592654", "0.0", "0.0", "base_link", "laser"],
    # )
    # ld.add_action(static_laser_tf)

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description,{"use_sim_time": True}],
    )
    ld.add_action(robot_state_publisher)

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )
    ld.add_action(twist_mux)

    # ros2_control using FakeSystem as hardware
    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("moveit_resources_robot_moveit_config"),
    #     "config",
    #     "ros2_controllers.yaml",
    # )
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_description, ros2_controllers_path],
    #     output="both",
    # )
    # ld.add_action(ros2_control_node)

    robot_ignition_control_node = Node(
        package="robot_ignition_control",
        executable="robot_ignition_control_action_server",
        name="robot_ignition_control",
        output="screen",
    )
    ld.add_action(robot_ignition_control_node)

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    ld.add_action(joint_state_broadcaster_spawner)

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_arm_controller", "-c", "/controller_manager"],
    )
    ld.add_action(arm_controller_spawner)

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "-c", "/controller_manager"],
    )
    ld.add_action(hand_controller_spawner)

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "-c", "/controller_manager"],
    )
    ld.add_action(diff_drive_spawner)

    # odom2tf = Node(
    #     package="robot_ignition",
    #     executable="odom2tf",
    #     name="odom2tf",
    #     output="screen",
    # )
    # ld.add_action(odom2tf)

    # Publish static tfs
    # robot_sticky_static_tf_publisher = Node(
    #     package="robot_ignition",
    #     executable="robot_sticky_static_tf_publisher",
    #     name="robot_sticky_static_tf_publisher",
    #     output="screen",
    # )
    # ld.add_action(robot_sticky_static_tf_publisher)

    # pkg_robot_ignition = get_package_share_directory('robot_ignition')
    # navigation = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource(
    #     os.path.join(pkg_robot_ignition, 'launch', 'navigation.launch.py')), launch_arguments={'use_sim_time': 'true', 'slam': 'False'}.items())    
    # ld.add_action(navigation)

# ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/ahmed/moveit2/ws_moveit_humble_control/src/my_robot/robot_moveit_config/config/mapper_params_online_async.yaml use_sim_time:=true

    pkg_slam = get_package_share_directory('slam_toolbox')
    slam = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_slam, 'launch', 'online_async_launch.py')), launch_arguments={'use_sim_time': 'true', 'slam_params_file':'/home/ahmed/moveit2/ws_moveit_humble_control/src/my_robot/robot_moveit_config/config/mapper_params_online_async2.yaml'}.items())    
    ld.add_action(slam)

    # ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=/home/ahmed/moveit2/ws_moveit_humble_control/src/robot_ros2/robot_ignition/config/navigation.yaml
    # pkg_navigation = get_package_share_directory('moveit_resources_robot_moveit_config')
    # nav = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource(
    #     os.path.join(pkg_navigation, 'launch', 'navigation_launch.py')), launch_arguments={'use_sim_time': 'true'}.items())    
    # ld.add_action(nav)

    pkg_navigation = get_package_share_directory('nav2_bringup')
    nav = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_navigation, 'launch', 'navigation_launch.py')), launch_arguments={'use_sim_time': 'true', 'params_file': '/home/ahmed/moveit2/ws_moveit_humble_control/src/robot_ros2/robot_ignition/config/navigation.yaml'}.items())    
    ld.add_action(nav)

    gst_image_capture = Node(
        package="ros2_opencv",
        executable="camera_remap_node",
        name="camera_remap_node",
        output="screen",
    )
    ld.add_action(gst_image_capture)

    april_tag_package_name='apriltag_ros'
    april_tag_params = os.path.join(get_package_share_directory(april_tag_package_name),'cfg','test_tags_36h11.yaml')
    april_tag_node = Node(
            # prefix=["gdbserver localhost:3000"],
            package="apriltag_ros",
            executable="apriltag_node",
            parameters=[april_tag_params],
            remappings=[('/image_rect','/image/uncompressed'),
                        ('/camera_info','/camera_info')]
        )
    ld.add_action(april_tag_node)

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )
    ld.add_action(gazebo)
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
    ld.add_action(spawn_entity)

    return ld