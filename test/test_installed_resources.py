import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory


def test_public_resources_are_installed() -> None:
    package_share = Path(get_package_share_directory('joint_position_controller_server'))
    expected_resources = (
        'README.md',
        'LICENSE',
        'CONTRIBUTING.md',
        'config/example_joint_position_controller_server.yaml',
        'config/example_prismatic_joint_position_serial_driver.yaml',
        'launch/joint_position_controller_server.launch.py',
        'launch/prismatic_joint_position_serial_driver.launch.py',
    )

    for relative_path in expected_resources:
        assert package_share.joinpath(relative_path).is_file(), relative_path


def test_public_executables_are_installed() -> None:
    package_prefix = Path(get_package_prefix('joint_position_controller_server'))
    executable_dir = package_prefix / 'lib' / 'joint_position_controller_server'
    executable_names = (
        'joint_position_controller_server_node',
        'prismatic_joint_position_serial_driver_node',
        'joint_position_controller_client_node',
        'joint_position_controller_client_single_goal_node',
        'joint_position_controller_client_single_goal_sequential_node',
    )

    for executable_name in executable_names:
        executable = executable_dir / executable_name
        assert executable.is_file(), executable_name
        assert os.access(executable, os.X_OK), executable_name


def test_installed_launch_files_expose_their_arguments() -> None:
    launch_files = (
        'joint_position_controller_server.launch.py',
        'prismatic_joint_position_serial_driver.launch.py',
    )

    for launch_file in launch_files:
        result = subprocess.run(
            ['ros2', 'launch', 'joint_position_controller_server', launch_file, '--show-args'],
            capture_output=True,
            check=False,
            text=True,
            timeout=20,
        )
        output = result.stdout + result.stderr
        assert result.returncode == 0, output
        assert "'node_args'" in output
        assert "'params_file_allow_substs'" in output
