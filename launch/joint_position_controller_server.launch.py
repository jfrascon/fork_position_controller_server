import os
from pathlib import Path
from typing import Any, List

import ros2_launch_helpers as rlh
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity  # noqa


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='robot', description='Namepace'),
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(
                    get_package_share_directory('joint_position_controller_server'),
                    'config',
                    'example_joint_position_controller_server.yaml',
                ),
                description='YAML file with node parameters',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Use simulation clock if true',
            ),
            DeclareLaunchArgument('lower_limit', default_value='', description='Lower limit of the joint'),
            DeclareLaunchArgument('upper_limit', default_value='', description='Upper limit of the joint'),
            DeclareLaunchArgument(
                'position_tolerance', default_value='', description='Position tolerance for the controller'
            ),
            DeclareLaunchArgument(
                'command_publication_frequency',
                default_value='',
                description='Frequency to publish the command topic',
            ),
            DeclareLaunchArgument(
                'feedback_publication_frequency',
                default_value='',
                description='Frequency to publish the feedback topic',
            ),
            DeclareLaunchArgument(
                'execution_loop_frequency',
                default_value='',
                description='Frequency of the execution loop',
            ),
            DeclareLaunchArgument(
                'goal_timeout', default_value='', description='Timeout for the goal to be considered failed'
            ),
            DeclareLaunchArgument(
                'joint_state_timeout',
                default_value='',
                description='Timeout for the joint state to be considered stale',
            ),
            DeclareLaunchArgument('joint_name', default_value='', description='Name of the joint to control'),
            DeclareLaunchArgument(
                'command_topic', default_value='', description='Topic to subscribe for joint position commands'
            ),
            DeclareLaunchArgument(
                'joint_states_topic', default_value='', description='Topic to subscribe for joint states'
            ),
            DeclareLaunchArgument(
                'node_logging_options',
                default_value=rlh.default_logging_options_str(),
                description=rlh.LOGGING_OPTIONS_DESC,
            ),
            DeclareLaunchArgument(
                'node_options', default_value=rlh.default_node_options_str(), description=rlh.NODE_OPTIONS_DESC
            ),
            OpaqueFunction(function=launch_server_node),
        ]
    )


def launch_server_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    # If the params_file exists, load it as a ParameterFile.
    # If any parameter is also provided to this launch file, it takes precedence over the
    # params_file.
    # This allows to override specific parameters in the params_file without having to create a new
    # params file.
    parameters: List[Any] = []

    params_file = LaunchConfiguration('params_file').perform(ctx).strip()
    lower_limit = LaunchConfiguration('lower_limit').perform(ctx).strip()
    upper_limit = LaunchConfiguration('upper_limit').perform(ctx).strip()
    position_tolerance = LaunchConfiguration('position_tolerance').perform(ctx).strip()
    command_publication_frequency = LaunchConfiguration('command_publication_frequency').perform(ctx).strip()
    feedback_publication_frequency = LaunchConfiguration('feedback_publication_frequency').perform(ctx).strip()
    execution_loop_frequency = LaunchConfiguration('execution_loop_frequency').perform(ctx).strip()
    goal_timeout = LaunchConfiguration('goal_timeout').perform(ctx).strip()
    joint_state_timeout = LaunchConfiguration('joint_state_timeout').perform(ctx).strip()
    joint_name = LaunchConfiguration('joint_name').perform(ctx).strip()
    commands_topic = LaunchConfiguration('command_topic').perform(ctx).strip()
    joint_states_topic = LaunchConfiguration('joint_states_topic').perform(ctx).strip()

    # Add parameter file only if it's not empty.
    if params_file:
        if not Path(params_file).is_file():
            raise FileNotFoundError(f"Params file '{params_file}' does not exist. ")

        parameters.append(ParameterFile(params_file, allow_substs=True))

    if lower_limit:
        try:
            parameters.append({'lower_limit': float(lower_limit)})
        except ValueError as exc:
            raise ValueError(f"Invalid value for lower_limit: '{lower_limit}'. Must be a float.") from exc

    if upper_limit:
        try:
            parameters.append({'upper_limit': float(upper_limit)})
        except ValueError as exc:
            raise ValueError(f"Invalid value for upper_limit: '{upper_limit}'. Must be a float.") from exc

    if position_tolerance:
        try:
            parameters.append({'position_tolerance': float(position_tolerance)})
        except ValueError as exc:
            raise ValueError(f"Invalid value for position_tolerance: '{position_tolerance}'. Must be a float.") from exc

    if command_publication_frequency:
        try:
            parameters.append({'command_publication_frequency': float(command_publication_frequency)})
        except ValueError as exc:
            raise ValueError(
                f"Invalid value for command_publication_frequency: '{command_publication_frequency}'. Must be a float."
            ) from exc

    if feedback_publication_frequency:
        try:
            parameters.append({'feedback_publication_frequency': float(feedback_publication_frequency)})
        except ValueError as exc:
            raise ValueError(
                f"Invalid value for feedback_publication_frequency: '{feedback_publication_frequency}'. Must be a float."
            ) from exc

    if execution_loop_frequency:
        try:
            parameters.append({'execution_loop_frequency': float(execution_loop_frequency)})
        except ValueError as exc:
            raise ValueError(
                f"Invalid value for execution_loop_frequency: '{execution_loop_frequency}'. Must be a float."
            ) from exc

    if goal_timeout:
        try:
            parameters.append({'goal_timeout': float(goal_timeout)})
        except ValueError as exc:
            raise ValueError(f"Invalid value for goal_timeout: '{goal_timeout}'. Must be a float.") from exc

    if joint_state_timeout:
        try:
            parameters.append({'joint_state_timeout': float(joint_state_timeout)})
        except ValueError as exc:
            raise ValueError(
                f"Invalid value for joint_state_timeout: '{joint_state_timeout}'. Must be a float."
            ) from exc

    if joint_name:
        parameters.append({'joint_name': joint_name})

    if commands_topic:
        parameters.append({'command_topic': commands_topic})

    if joint_states_topic:
        parameters.append({'joint_states_topic': joint_states_topic})

    parameters.append({'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)})

    node_options = rlh.process_node_options(LaunchConfiguration('node_options').perform(ctx))
    node_name = str(node_options['name']) or 'joint_position_controller_server'

    if not rlh.is_valid_name(node_name):
        raise RuntimeError(f"The name of the node must be ASCII [A-Za-z0-9_] only: '{node_name}'")

    return [
        Node(
            package='joint_position_controller_server',
            executable='joint_position_controller_server_node',
            name=node_name,
            namespace=LaunchConfiguration('namespace'),
            parameters=parameters,
            ros_arguments=rlh.process_node_logging_options(LaunchConfiguration('node_logging_options').perform(ctx)),
            output=node_options['output'],
            emulate_tty=node_options['emulate_tty'],
            respawn=node_options['respawn'],
            respawn_delay=node_options['respawn_delay'],
        )
    ]
