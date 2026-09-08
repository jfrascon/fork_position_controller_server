import math

import pytest
import rclpy
from std_msgs.msg import Float64

from joint_position_controller_server import (
    prismatic_joint_position_serial_driver as driver_module,
)


class FakeSerial:
    instances: list['FakeSerial'] = []

    def __init__(self, **configuration: object) -> None:
        self.port = configuration['port']
        self.baudrate = configuration['baudrate']
        self.is_open = True
        self.incoming = b''
        self.fail_writes = False
        self.writes: list[bytes] = []
        self.instances.append(self)

    @property
    def in_waiting(self) -> int:
        return len(self.incoming)

    def read(self, size: int) -> bytes:
        data = self.incoming[:size]
        self.incoming = self.incoming[size:]
        return data

    def write(self, data: bytes) -> int:
        if self.fail_writes:
            raise driver_module.serial.SerialException('test write failure')
        self.writes.append(data)
        return len(data)

    def close(self) -> None:
        self.is_open = False


def make_driver(monkeypatch: pytest.MonkeyPatch, parameters: dict[str, object] | None = None):
    FakeSerial.instances.clear()
    monkeypatch.setattr(driver_module.serial, 'Serial', FakeSerial)

    ros_arguments = ['--ros-args']
    for name, value in (parameters or {}).items():
        ros_arguments.extend(['-p', f'{name}:={value}'])

    rclpy.init(args=ros_arguments)
    try:
        return driver_module.PrismaticJointPositionSerialDriver()
    except Exception:
        rclpy.shutdown()
        raise


def destroy_driver(driver: driver_module.PrismaticJointPositionSerialDriver) -> None:
    driver.destroy_node()
    rclpy.shutdown()


def test_driver_opens_and_closes_the_configured_serial_port(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    driver = make_driver(monkeypatch, {'port': 'test_port', 'baudrate': 115200})
    serial_port = FakeSerial.instances[0]

    assert serial_port.port == 'test_port'
    assert serial_port.baudrate == 115200

    destroy_driver(driver)

    assert serial_port.is_open is False
    assert serial_port.writes == [b'0']


def test_driver_closes_the_port_when_the_shutdown_stop_command_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    driver = make_driver(monkeypatch)
    serial_port = FakeSerial.instances[0]
    serial_port.fail_writes = True

    destroy_driver(driver)

    assert serial_port.is_open is False


@pytest.mark.parametrize(
    'parameters',
    [
        {'lower_limit': 0.2, 'upper_limit': 0.1},
        {'convergence_threshold': -0.01},
        {'execution_loop_frequency': 0.0},
        {'low_pass_filter_coeff': 1.1},
        {'command_topic': "''"},
        {'joint_states_topic': "''"},
        {'port': "''"},
        {'baudrate': 0},
        {'data_bits': 4},
        {'flow_control': 'invalid'},
        {'parity': 'invalid'},
        {'stop_bits': 3.0},
    ],
)
def test_invalid_parameters_fail_before_opening_serial_port(
    monkeypatch: pytest.MonkeyPatch, parameters: dict[str, object]
) -> None:
    with pytest.raises(ValueError):
        make_driver(monkeypatch, parameters)

    assert FakeSerial.instances == []


def test_target_callback_rejects_non_finite_and_out_of_range_positions(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    driver = make_driver(monkeypatch)

    driver._target_pos_callback(Float64(data=math.nan))
    assert driver._target_position is None

    driver._target_pos_callback(Float64(data=0.3))
    assert driver._target_position is None

    driver._target_pos_callback(Float64(data=0.125))
    assert driver._target_position == pytest.approx(0.125)
    assert driver._target_pos_mm == pytest.approx(125.0)

    destroy_driver(driver)


def test_inverse_ir_calibration_accepts_values_between_both_endpoints(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    driver = make_driver(monkeypatch, {'lower_ir_value': 0.3, 'upper_ir_value': 0.1})
    serial_port = FakeSerial.instances[0]
    serial_port.incoming = b'0,200\n'

    driver._update()

    assert driver._filtered_current_pos_mm == pytest.approx(100.0)
    assert serial_port.writes == []

    destroy_driver(driver)


def test_valid_target_produces_the_expected_single_byte_command(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    driver = make_driver(monkeypatch)
    serial_port = FakeSerial.instances[0]
    driver._target_pos_callback(Float64(data=0.15))
    serial_port.incoming = b'0,200\n'

    driver._update()

    assert serial_port.writes == [b'1']

    destroy_driver(driver)


def test_stop_command_is_retried_after_a_serial_write_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    driver = make_driver(monkeypatch)
    serial_port = FakeSerial.instances[0]
    driver._target_pos_callback(Float64(data=0.1))

    serial_port.fail_writes = True
    serial_port.incoming = b'1,200\n'
    driver._update()

    assert driver._target_position_reached is False
    assert serial_port.writes == []

    serial_port.fail_writes = False
    serial_port.incoming = b'1,200\n'
    driver._update()

    assert driver._target_position_reached is True
    assert serial_port.writes == [b'0']

    destroy_driver(driver)
