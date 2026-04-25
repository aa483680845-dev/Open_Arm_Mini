from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	joy_node = Node(
		package='joy',
		executable='joy_node',
		name='joy_node',
		parameters=[{'sticky_buttons': True},
					{'autorepeat_rate': 150.0},],
		output='screen'
	)

	pinocchio_inverse = Node(
		package='robot_pinocchio',
		executable='pinocchio_main',
		name='pinocchio_main',
		output='screen',
	)

	return LaunchDescription([
		joy_node,
		pinocchio_inverse,
	])

