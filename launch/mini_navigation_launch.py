# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('f112th_sim_2401_echo')

    nav2_yaml = os.path.join(bringup_dir, 'config', 'mini_nav2_params.yaml')
    map_file = os.path.join(bringup_dir, 'maps', 'map_save_nworld.yaml')

    # state machine for nodes
    lifecycle_nodes = [ 'map_server',
                        'amcl',
                        'planner_server',
                        'controller_server',
                        'nav2_behaviors',
                        'bt_navigator'
                        ]
    
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]


    #
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    

    # Node definition

    map_server_node = Node(package='nav2_map_server',
                            executable='map_server',
                            name='map_server',
                            output='screen',
                            parameters=[nav2_yaml, {'yaml_filename': map_file}])
    
    amcl_node = Node(package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[nav2_yaml])


    # Create instances of your path planning, obstacle avoidance, and motion control nodes
    nav2_controller_server_node = Node(package='nav2_controller',
                                        executable='controller_server',
                                        output='screen',
                                        parameters=[nav2_yaml],
                                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')])
   
    planner_server_node = Node(package='nav2_planner',
                                executable='planner_server',
                                name='planner_server',
                                output='screen',
                                parameters=[nav2_yaml],
                                remappings=remappings)
   

    behavior_server_node = Node(package='nav2_behaviors',
                                executable='behavior_server',
                                name='behavior_server',
                                output='screen',
                                parameters=[nav2_yaml],
                                remappings=remappings)
    
    bt_navigator_node = Node(package='nav2_bt_navigator',
                            executable='bt_navigator',
                            name='bt_navigator',
                            output='screen',
                            parameters=[nav2_yaml],
                            remappings=remappings)

    lifecycle_manager_navigation_node = Node(package='nav2_lifecycle_manager',
                                            executable='lifecycle_manager',
                                            name='lifecycle_manager_navigation',
                                            output='screen',
                                            parameters=[{'use_sim_time': True},
                                                        {'autostart': True},
                                                        {'node_names': lifecycle_nodes}])

   
    return LaunchDescription([
		stdout_linebuf_envvar,
		map_server_node,
        amcl_node,
		nav2_controller_server_node,
        planner_server_node,
		behavior_server_node,
		bt_navigator_node,
		lifecycle_manager_navigation_node,
	])