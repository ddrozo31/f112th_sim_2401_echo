# library to move between files and folders in the O.S.
import os

from ament_index_python.packages import get_package_share_directory

# libraries to define the Launch file and Function
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():


	# Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
	# !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

	package_name='f112th_sim_2401_echo' #<--- CHANGE ME

	agent_ns = LaunchConfiguration('agent_ns')

	agent_ns_launch_arg = DeclareLaunchArgument(
        'agent_ns',
        default_value=''
    )


	rsp = IncludeLaunchDescription(
				PythonLaunchDescriptionSource([os.path.join(
					get_package_share_directory(package_name),'launch','rsp.launch.py'
				)]), launch_arguments={'agent_ns':agent_ns,'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
	)

	# Include the Gazebo launch file, provided by the gazebo_ros package
	gazebo = IncludeLaunchDescription(
				PythonLaunchDescriptionSource([os.path.join(
					get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
			 )

	# Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
	spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
					 	namespace = agent_ns,
						arguments=['-topic', 'robot_description',
								   '-entity', 'my_bot'],
						output='screen')


	# Launch the Diff_Controller
	diff_drive_spawner = Node(
		package='controller_manager', 
		namespace = agent_ns,
		executable='spawner', 
		arguments=['diff_cont'])
		
		# Launch the Joint_Broadcaster
	joint_broad_spawner = Node(
		package='controller_manager',
		namespace = agent_ns, 
		executable='spawner', 
		arguments=['joint_broad'])
	
	joystick = IncludeLaunchDescription(
				PythonLaunchDescriptionSource([os.path.join(
					get_package_share_directory(package_name),'launch','joystick.launch.py'
				)]), launch_arguments={'agent_ns':agent_ns,'use_sim_time': 'true'}.items()
	)


	twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
	
	twist_mux_node = Node(package='twist_mux', 
					namespace = agent_ns,
					executable='twist_mux',
					parameters=[twist_mux_params,{'use_sim_time': True}],
					remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
	)
	

	# Launch them all!
	return LaunchDescription([
		agent_ns_launch_arg,
		rsp,
		joystick,
        twist_mux_node,
		gazebo,
		spawn_entity,
		diff_drive_spawner,
		joint_broad_spawner
	])