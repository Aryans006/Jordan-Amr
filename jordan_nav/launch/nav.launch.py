import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
	pkg_jordan_description = get_package_share_directory('jordan_nav')
	pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

	use_sim_time = LaunchConfiguration('use_sim_time', default='true')
	with_rviz = LaunchConfiguration('with_rviz', default='true')
	map_yaml_file = LaunchConfiguration('map',
					default=os.path.join(pkg_jordan_description, 'map', 'Map_plain_save.yaml'))
	nav2_config_file = LaunchConfiguration('params', 
						default=os.path.join(pkg_jordan_description, 'config', 'nav2_params.yaml'))


	nav2_bringup_launch_file = IncludeLaunchDescription(
					PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
					launch_arguments={
						'map': map_yaml_file,
						'use_sim_time': use_sim_time,
						'params_file': nav2_config_file}.items(),
				)

	
	return LaunchDescription([
		DeclareLaunchArgument('map',
					default_value=map_yaml_file,
					description="/home/aryan/jordan/src/Jordan-Amr/jordan_nav/map/Map_plain_save.yaml"),
		DeclareLaunchArgument('params_file',
		     		default_value=nav2_config_file,
		     		description="/home/aryan/jordan/src/Jordan-Amr/jordan_nav/config/nav2_params.yaml"),
		DeclareLaunchArgument('use_sim_time',
                                default_value=use_sim_time,
                                description="Use sim clock or not"),
		nav2_bringup_launch_file,
		])