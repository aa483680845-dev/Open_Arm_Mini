from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    use_mojoco = LaunchConfiguration('use_mojoco')

    my_robot_path = PathJoinSubstitution([
        FindPackageShare("robot_description"),
        "urdf",
        "my_robot.xacro"
    ])

    # urdf_path = PathJoinSubstitution([
    #     FindPackageShare("robot_description"),
    #     "urdf",
    #     "robot_1.urdf"
    # ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', my_robot_path]), value_type=str),
        }]
    )
     
    controller_manager_node_real = Node(
        package='controller_manager',
        executable='ros2_control_node',
        condition = UnlessCondition(use_mojoco),
        parameters=[
            {'robot_description': ParameterValue(Command(['xacro ', my_robot_path]), value_type=str)},
            PathJoinSubstitution([
                FindPackageShare("robot_bringup"),
                "config",
                "ros2_controllers.yaml"
            ]),
        ]
        
    )
    controller_manager_node_mojoco = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        condition = IfCondition(use_mojoco),
        parameters=[
        {'robot_description': ParameterValue(Command(['xacro ', my_robot_path]), value_type=str)},
         {"use_sim_time": True},
            PathJoinSubstitution([
                FindPackageShare("robot_bringup"),
                "config",
                "ros2_controllers.yaml"
            ])
        ]
    )
    # 控制器加载节点
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    arm_MIT_command_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_MIT_command_controller']
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare("robot_bringup"),
            "rviz",
            "control_rviz.rviz"
        ])],
        output='screen',
        # parameters=rviz_parameters
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_mojoco',
            default_value='false',
            description='Whether to use Mojoco or not'
        ),

        robot_state_publisher_node,

        controller_manager_node_real,
        controller_manager_node_mojoco,

        joint_state_broadcaster_spawner,
        arm_MIT_command_controller_spawner,
        rviz_node,
    ])


