import os
from pathlib import Path
from typing import Any, List

import ros2_launch_helpers as rlh
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace', default_value='robot', description='namespace where the node is launched'
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(
                    get_package_share_directory('joint_position_controller_server'),
                    'config',
                    'example_prismatic_joint_position_serial_driver.yaml',
                ),
                description='YAML file with node parameters',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Use simulation clock if true',
            ),
            DeclareLaunchArgument(
                'port', default_value='', description='Serial device path. If empty, use the value from params_file.'
            ),
            DeclareLaunchArgument(
                'baudrate', default_value='', description='Serial baudrate. If empty, use the value from params_file.'
            ),
            DeclareLaunchArgument(
                'data_bits', default_value='', description='Serial data bits. If empty, use the value from params_file.'
            ),
            DeclareLaunchArgument(
                'flow_control',
                default_value='',
                choices=['', 'none', 'hardware', 'software'],
                description='Serial flow control mode. If empty, use the value from params_file.',
            ),
            DeclareLaunchArgument(
                'parity',
                default_value='',
                choices=['', 'none', 'even', 'odd', 'mark', 'space'],
                description='Serial parity mode. If empty, use the value from params_file.',
            ),
            DeclareLaunchArgument(
                'stop_bits',
                default_value='',
                choices=['', '1', '1.5', '2'],
                description='Serial stop bits. If empty, use the value from params_file.',
            ),
            DeclareLaunchArgument(
                'lower_limit',
                default_value='',
                description='Lower joint position limit. If empty, use the value from params_file.',
            ),
            DeclareLaunchArgument(
                'upper_limit',
                default_value='',
                description='Upper joint position limit. If empty, use the value from params_file.',
            ),
            DeclareLaunchArgument(
                'lower_ir_value',
                default_value='',
                description='Lower IR value threshold. If empty, use the value from params_file.',
            ),
            DeclareLaunchArgument(
                'upper_ir_value',
                default_value='',
                description='Upper IR value threshold. If empty, use the value from params_file.',
            ),
            DeclareLaunchArgument(
                'convergence_threshold',
                default_value='',
                description='Convergence threshold used to decide when the joint should stop. '
                'If empty, use the value from params_file.',
            ),
            DeclareLaunchArgument(
                'execution_loop_frequency',
                default_value='',
                description='Rate in Hz used by the timer that updates the serial connection. '
                'If empty, use the value from params_file.',
            ),
            DeclareLaunchArgument(
                'low_pass_filter_coeff',
                default_value='',
                description='Low-pass filter coefficient in the inclusive range [0.0, 1.0]. '
                'If empty, use the value from params_file.',
            ),
            DeclareLaunchArgument('joint_name', default_value='', description='Name of the controlled joint'),
            DeclareLaunchArgument(
                'command_topic',
                default_value='',
                description='Topic used to receive joint target positions as std_msgs/msg/Float64. '
                'If empty, use the value from params_file.',
            ),
            DeclareLaunchArgument(
                'joint_states_topic', default_value='', description='Topic to publish for the joint state'
            ),
            # DeclareLaunchArgument(
            #     'position_topic',
            #     default_value='',
            #     description='Topic used to publish the joint position as std_msgs/msg/Float64. '
            #     'If empty, use the value from params_file.',
            # ),
            # DeclareLaunchArgument(
            #     'sensor_value_topic',
            #     default_value='',
            #     description='Topic used to publish the raw joint sensor value as std_msgs/msg/Int32. '
            #     'If empty, use the value from params_file.',
            # ),
            DeclareLaunchArgument(
                'node_options', default_value=rlh.default_node_options_str(), description=rlh.NODE_OPTIONS_DESC
            ),
            DeclareLaunchArgument(
                'node_logging_options',
                default_value=rlh.default_logging_options_str(),
                description=rlh.LOGGING_OPTIONS_DESC,
            ),
            OpaqueFunction(function=launch_prismatic_joint_position_serial_driver_node),
        ]
    )


def launch_prismatic_joint_position_serial_driver_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    parameters: List[Any] = []

    params_file = LaunchConfiguration('params_file').perform(ctx)
    port = LaunchConfiguration('port').perform(ctx)
    baudrate = LaunchConfiguration('baudrate').perform(ctx)
    data_bits = LaunchConfiguration('data_bits').perform(ctx)
    flow_control = LaunchConfiguration('flow_control').perform(ctx)
    parity = LaunchConfiguration('parity').perform(ctx)
    stop_bits = LaunchConfiguration('stop_bits').perform(ctx)
    lower_limit = LaunchConfiguration('lower_limit').perform(ctx)
    upper_limit = LaunchConfiguration('upper_limit').perform(ctx)
    lower_ir_value = LaunchConfiguration('lower_ir_value').perform(ctx)
    upper_ir_value = LaunchConfiguration('upper_ir_value').perform(ctx)
    convergence_threshold = LaunchConfiguration('convergence_threshold').perform(ctx)
    execution_loop_frequency = LaunchConfiguration('execution_loop_frequency').perform(ctx)
    low_pass_filter_coeff = LaunchConfiguration('low_pass_filter_coeff').perform(ctx)
    joint_name = LaunchConfiguration('joint_name').perform(ctx)
    command_topic = LaunchConfiguration('command_topic').perform(ctx)
    joint_states_topic = LaunchConfiguration('joint_states_topic').perform(ctx)
    # position_topic = LaunchConfiguration('position_topic').perform(ctx)
    # sensor_value_topic = LaunchConfiguration('sensor_value_topic').perform(ctx)

    if params_file:
        if not Path(params_file).is_file():
            raise FileNotFoundError(f"Params file '{params_file}' does not exist.")
        parameters.append(ParameterFile(params_file, allow_substs=True))

    if port:
        parameters.append({'port': port})

    if baudrate:
        try:
            parameters.append({'baudrate': int(baudrate)})
        except ValueError as exc:
            raise ValueError(f"Invalid value for baudrate: '{baudrate}'. Expected an integer.") from exc

    if data_bits:
        try:
            parameters.append({'data_bits': int(data_bits)})
        except ValueError as exc:
            raise ValueError(f"Invalid value for data_bits: '{data_bits}'. Expected an integer.") from exc

    if flow_control:
        parameters.append({'flow_control': flow_control})

    if parity:
        parameters.append({'parity': parity})

    if stop_bits:
        try:
            parameters.append({'stop_bits': float(stop_bits)})
        except ValueError as exc:
            raise ValueError(f"Invalid value for stop_bits: '{stop_bits}'. Expected 1, 1.5 or 2.") from exc

    if lower_limit:
        try:
            parameters.append({'lower_limit': float(lower_limit)})
        except ValueError as exc:
            raise ValueError(f"Invalid value for lower_limit: '{lower_limit}'. Expected a float.") from exc

    if upper_limit:
        try:
            parameters.append({'upper_limit': float(upper_limit)})
        except ValueError as exc:
            raise ValueError(f"Invalid value for upper_limit: '{upper_limit}'. Expected a float.") from exc

    if lower_ir_value:
        try:
            parameters.append({'lower_ir_value': int(lower_ir_value)})
        except ValueError as exc:
            raise ValueError(f"Invalid value for lower_ir_value: '{lower_ir_value}'. Expected an integer.") from exc

    if upper_ir_value:
        try:
            parameters.append({'upper_ir_value': int(upper_ir_value)})
        except ValueError as exc:
            raise ValueError(f"Invalid value for upper_ir_value: '{upper_ir_value}'. Expected an integer.") from exc

    if convergence_threshold:
        try:
            parameters.append({'convergence_threshold': float(convergence_threshold)})
        except ValueError as exc:
            raise ValueError(
                f"Invalid value for convergence_threshold: '{convergence_threshold}'. Expected a float."
            ) from exc

    if execution_loop_frequency:
        try:
            parameters.append({'execution_loop_frequency': float(execution_loop_frequency)})
        except ValueError as exc:
            raise ValueError(
                f"Invalid value for execution_loop_frequency: '{execution_loop_frequency}'. Expected a float."
            ) from exc

    if low_pass_filter_coeff:
        try:
            parameters.append({'low_pass_filter_coeff': float(low_pass_filter_coeff)})
        except ValueError as exc:
            raise ValueError(
                f"Invalid value for low_pass_filter_coeff: '{low_pass_filter_coeff}'. Expected a float."
            ) from exc

    if joint_name:
        parameters.append({'joint_name': joint_name})

    if command_topic:
        parameters.append({'command_topic': command_topic})

    if joint_states_topic:
        parameters.append({'joint_states_topic': joint_states_topic})

    # if position_topic:
    #     parameters.append({'position_topic': position_topic})

    # if sensor_value_topic:
    #     parameters.append({'sensor_value_topic': sensor_value_topic})

    parameters.append({'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)})

    node_options = rlh.process_node_options(LaunchConfiguration('node_options').perform(ctx))
    node_name = str(node_options['name']) or 'prismatic_joint_position_serial_driver'

    if not rlh.is_valid_name(node_name):
        raise RuntimeError(f"The name of the node must be ASCII [A-Za-z0-9_] only: '{node_name}'")

    return [
        Node(
            package='joint_position_controller_server',
            executable='prismatic_joint_position_serial_driver_node',
            namespace=LaunchConfiguration('namespace'),
            name=node_name,
            parameters=parameters,
            ros_arguments=rlh.process_node_logging_options(LaunchConfiguration('node_logging_options').perform(ctx)),
            output=node_options['output'],
            emulate_tty=node_options['emulate_tty'],
            respawn=node_options['respawn'],
            respawn_delay=node_options['respawn_delay'],
        )
    ]
