import importlib.util
from pathlib import Path
from types import ModuleType

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest
import yaml

PACKAGE_DIR = Path(__file__).parents[1]
STANDARD_LAUNCH_ARGUMENTS = {
    'namespace',
    'params_file',
    'params_file_allow_substs',
    'use_sim_time',
    'node_args',
}


def load_launch_module(filename: str) -> ModuleType:
    path = PACKAGE_DIR / 'launch' / filename
    spec = importlib.util.spec_from_file_location(filename.replace('.', '_'), path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    'launch_file',
    [
        'joint_position_controller_server.launch.py',
        'prismatic_joint_position_serial_driver.launch.py',
    ],
)
def test_launch_files_expose_the_current_parameter_file_contract(launch_file: str) -> None:
    module = load_launch_module(launch_file)
    launch_description = module.generate_launch_description()
    declarations = {
        action.name: action
        for action in launch_description.entities
        if isinstance(action, DeclareLaunchArgument)
    }

    assert set(declarations) == STANDARD_LAUNCH_ARGUMENTS
    assert declarations['params_file'].default_value is None
    assert declarations['params_file_allow_substs'].default_value is None
    assert declarations['use_sim_time'].default_value is None

    context = LaunchContext()
    declarations['node_args'].visit(context)
    assert context.launch_configurations['node_args'] == (
        '{"output":"both","ros_arguments":["--log-level","info"]}'
    )


@pytest.mark.parametrize(
    'config_file',
    [
        'example_joint_position_controller_server.yaml',
        'example_prismatic_joint_position_serial_driver.yaml',
    ],
)
def test_launch_owns_use_sim_time(config_file: str) -> None:
    parameters = yaml.safe_load((PACKAGE_DIR / 'config' / config_file).read_text(encoding='utf-8'))
    node_parameters = next(iter(parameters.values()))['ros__parameters']

    assert 'use_sim_time' not in node_parameters
