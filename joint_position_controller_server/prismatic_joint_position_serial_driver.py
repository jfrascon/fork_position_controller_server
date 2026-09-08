from __future__ import annotations

import math
from typing import Final

from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
from std_msgs.msg import Float64


class PrismaticJointPositionSerialDriver(Node):
    """
    Connect one ROS prismatic-joint position command to the serial motor protocol.

    ROS target positions and published JointState positions use meters.
    The serial device reports ``<motor_status>,<sensor_distance_mm>`` frames terminated by a
    newline. The driver sends one ASCII byte: ``0`` stops, ``1`` moves toward the upper joint
    limit, and ``2`` moves toward the lower joint limit.
    """

    FLOW_CONTROL_NONE: Final[str] = 'none'
    FLOW_CONTROL_HARDWARE: Final[str] = 'hardware'
    FLOW_CONTROL_SOFTWARE: Final[str] = 'software'
    VALID_FLOW_CONTROLS: Final[set[str]] = {
        FLOW_CONTROL_NONE,
        FLOW_CONTROL_HARDWARE,
        FLOW_CONTROL_SOFTWARE,
    }

    VALID_PARITIES: Final[dict[str, str]] = {
        'none': serial.PARITY_NONE,
        'even': serial.PARITY_EVEN,
        'odd': serial.PARITY_ODD,
        'mark': serial.PARITY_MARK,
        'space': serial.PARITY_SPACE,
    }

    VALID_STOP_BITS: Final[dict[float, float]] = {
        1.0: serial.STOPBITS_ONE,
        1.5: serial.STOPBITS_ONE_POINT_FIVE,
        2.0: serial.STOPBITS_TWO,
    }

    VALID_BYTE_SIZES: Final[dict[int, int]] = {
        5: serial.FIVEBITS,
        6: serial.SIXBITS,
        7: serial.SEVENBITS,
        8: serial.EIGHTBITS,
    }

    def __init__(self) -> None:
        super().__init__('prismatic_joint_position_serial_driver')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('data_bits', 8)
        self.declare_parameter('flow_control', self.FLOW_CONTROL_NONE)
        self.declare_parameter('parity', 'none')
        self.declare_parameter('stop_bits', 1.0)
        self.declare_parameter('lower_limit', 0.0)
        self.declare_parameter('upper_limit', 0.2)
        self.declare_parameter('lower_ir_value', 0.1)
        self.declare_parameter('upper_ir_value', 0.3)
        self.declare_parameter('convergence_threshold', 0.01)
        self.declare_parameter('execution_loop_frequency', 10.0)
        self.declare_parameter('low_pass_filter_coeff', 0.10)
        self.declare_parameter('joint_name', 'joint')
        self.declare_parameter('command_topic', 'joint_commands/position')
        self.declare_parameter('joint_states_topic', 'joint_states')

        lower_limit = self.get_parameter('lower_limit').get_parameter_value().double_value
        upper_limit = self.get_parameter('upper_limit').get_parameter_value().double_value
        lower_ir_value = self.get_parameter('lower_ir_value').get_parameter_value().double_value
        upper_ir_value = self.get_parameter('upper_ir_value').get_parameter_value().double_value
        self._convergence_threshold = (
            self.get_parameter('convergence_threshold').get_parameter_value().double_value
        )

        finite_parameters = {
            'lower_limit': lower_limit,
            'upper_limit': upper_limit,
            'lower_ir_value': lower_ir_value,
            'upper_ir_value': upper_ir_value,
            'convergence_threshold': self._convergence_threshold,
        }

        for parameter_name, parameter_value in finite_parameters.items():
            if not math.isfinite(parameter_value):
                raise ValueError(f"Parameter '{parameter_name}' must be finite.")

        if lower_limit >= upper_limit:
            raise ValueError("Parameter 'lower_limit' must be less than 'upper_limit'.")

        if lower_ir_value == upper_ir_value:
            raise ValueError('Parameters lower_ir_value and upper_ir_value must be different.')

        if self._convergence_threshold < 0.0:
            raise ValueError("Parameter 'convergence_threshold' must be non-negative.")

        # ROS-facing positions use meters, while the serial protocol uses millimeters.
        self._lower_limit_mm = lower_limit * 1000.0
        self._upper_limit_mm = upper_limit * 1000.0
        self._lower_ir_value_mm = lower_ir_value * 1000.0
        self._upper_ir_value_mm = upper_ir_value * 1000.0
        self._convergence_threshold_mm = self._convergence_threshold * 1000.0

        position_range_mm = self._upper_limit_mm - self._lower_limit_mm
        ir_range_mm = self._upper_ir_value_mm - self._lower_ir_value_mm

        # Convert a sensor distance to its corresponding joint position.
        # position = slope * (ir_value - lower_ir_value) + lower_limit
        self._ir_value_to_position_slope = position_range_mm / ir_range_mm

        # Keep the target in both protocol and ROS units to avoid repeated conversions.
        self._target_pos_mm: float | None = None
        self._target_position: float | None = None

        self._target_position_reached: bool = False
        self._serial: serial.Serial | None = None
        self._serial_read_buffer = b''
        self._joint_state_publisher = None
        self._joint_name = ''
        self._filtered_current_pos_mm: float | None = None
        self._low_pass_filter_coeff = (
            self.get_parameter('low_pass_filter_coeff').get_parameter_value().double_value
        )
        self._update_logger = self.get_logger().get_child('update')
        self._serial_update_logger = self.get_logger().get_child('serial_update')
        self._target_position_logger = self.get_logger().get_child('target_position')
        self._update_sequence = 0
        self._update_period_s = 0.0

        joint_name = self.get_parameter('joint_name').get_parameter_value().string_value.strip()
        command_topic = (
            self.get_parameter('command_topic').get_parameter_value().string_value.strip()
        )
        joint_states_topic = (
            self.get_parameter('joint_states_topic').get_parameter_value().string_value.strip()
        )

        if not joint_name:
            raise ValueError("Parameter 'joint_name' must not be empty.")

        if not command_topic:
            raise ValueError("Parameter 'command_topic' must not be empty.")

        if not joint_states_topic:
            raise ValueError("Parameter 'joint_states_topic' must not be empty.")

        # This frequency controls serial reads, command updates, and JointState publication.
        execution_loop_frequency = (
            self.get_parameter('execution_loop_frequency').get_parameter_value().double_value
        )

        if not math.isfinite(execution_loop_frequency) or execution_loop_frequency <= 0.0:
            raise ValueError("Parameter 'execution_loop_frequency' must be finite and positive.")

        if (
            not math.isfinite(self._low_pass_filter_coeff)
            or not 0.0 <= self._low_pass_filter_coeff <= 1.0
        ):
            raise ValueError(
                "Parameter 'low_pass_filter_coeff' must be in the inclusive range [0.0, 1.0]."
            )

        self._joint_name = joint_name
        self._subscriber = self.create_subscription(
            Float64, command_topic, self._target_pos_callback, 10
        )
        self._joint_state_publisher = self.create_publisher(JointState, joint_states_topic, 10)

        self._update_period_s = 1.0 / execution_loop_frequency
        self._timer = self.create_timer(self._update_period_s, self._update)
        self._open_serial_port()

        self.get_logger().get_child('init').info(
            f"Updating joint serial connection on '{self._serial.port}' "
            f"at '{self._serial.baudrate}' baud"
        )

    def destroy_node(self) -> bool:
        timer = getattr(self, '_timer', None)
        if timer is not None:
            timer.cancel()

        self._close_serial_port()

        return super().destroy_node()

    def _target_pos_callback(self, msg: Float64) -> None:
        if not math.isfinite(msg.data):
            self._target_position_logger.warning('Ignoring a non-finite target position.')
            return

        target_pos_mm = msg.data * 1000.0
        if target_pos_mm < self._lower_limit_mm or target_pos_mm > self._upper_limit_mm:
            self._target_position_logger.warning(
                f'Ignoring target position {msg.data} m because it is outside the configured '
                'range '
                f'[{self._lower_limit_mm / 1000.0}, {self._upper_limit_mm / 1000.0}] m.'
            )
            return

        if self._target_position is not None:
            dist = abs(msg.data - self._target_position)

            if dist <= self._convergence_threshold:
                return

        self._target_position = msg.data
        self._target_pos_mm = target_pos_mm
        self._target_position_reached = False

    def _open_serial_port(self) -> None:
        self._close_serial_port()

        port = self.get_parameter('port').get_parameter_value().string_value.strip()
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        data_bits = self.get_parameter('data_bits').get_parameter_value().integer_value
        flow_control = (
            self.get_parameter('flow_control').get_parameter_value().string_value.strip().lower()
        )
        parity = self.get_parameter('parity').get_parameter_value().string_value.strip().lower()
        stop_bits = self.get_parameter('stop_bits').get_parameter_value().double_value

        self._validate_serial_configuration(
            port, baudrate, data_bits, flow_control, parity, stop_bits
        )

        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=self.VALID_BYTE_SIZES[data_bits],
            parity=self.VALID_PARITIES[parity],
            stopbits=self.VALID_STOP_BITS[stop_bits],
            timeout=0.0,
            xonxoff=flow_control == self.FLOW_CONTROL_SOFTWARE,
            rtscts=flow_control == self.FLOW_CONTROL_HARDWARE,
            dsrdtr=False,
            write_timeout=0.0,
        )

    def _close_serial_port(self) -> None:
        if self._serial is not None and self._serial.is_open:
            try:
                # Stop the motor before releasing the connection.
                # Closing a serial port does not guarantee that the firmware stops by itself.
                self._serial.write(b'0')
            except serial.SerialException as exc:
                self.get_logger().get_child('serial_shutdown').error(
                    f'Failed to send the stop command before closing the serial port: {exc}'
                )
            finally:
                self._serial.close()
        self._serial = None

    def _validate_serial_configuration(
        self,
        port: str,
        baudrate: int,
        data_bits: int,
        flow_control: str,
        parity: str,
        stop_bits: float,
    ) -> None:
        if not port:
            raise ValueError("Parameter 'port' must not be empty.")

        if baudrate <= 0:
            raise ValueError("Parameter 'baudrate' must be positive.")

        if data_bits not in self.VALID_BYTE_SIZES:
            raise ValueError(
                f'Unsupported data_bits value: {data_bits}. Expected one of 5, 6, 7, 8.'
            )

        if flow_control not in self.VALID_FLOW_CONTROLS:
            raise ValueError(
                f"Unsupported flow_control value: '{flow_control}'. "
                "Expected 'none', 'hardware' or 'software'."
            )

        if parity not in self.VALID_PARITIES:
            raise ValueError(
                f"Unsupported parity value: '{parity}'. "
                "Expected 'none', 'even', 'odd', 'mark' or 'space'."
            )

        if stop_bits not in self.VALID_STOP_BITS:
            raise ValueError(f"Unsupported stop_bits value: '{stop_bits}'. Expected 1, 1.5 or 2.")

    def _update(self) -> None:
        self._update_sequence += 1
        update_id = self._update_sequence

        if self._serial is None or not self._serial.is_open:
            self._update_logger.error(
                f'Update #{update_id}: Serial port is not open. Skipping serial update.'
            )
            return

        read_result = self._read()

        # No valid data read from the serial port, skipping update.
        if read_result is None:
            return

        # Expected output coming from the serial communication:
        # STATE, SENSOR_VALUE\n
        # where STATE is:
        # 0: stopped
        # 1: moving up
        # 2: moving down
        motor_status, ir_value_mm = read_result

        if motor_status is None or motor_status not in {'0', '1', '2'} or ir_value_mm is None:
            self._update_logger.warning(
                f'Update #{update_id}: Ignored invalid frame. status = {motor_status!r}, '
                f'ir_value = {ir_value_mm!r} mm'
            )
            return

        minimum_ir_value_mm = min(self._lower_ir_value_mm, self._upper_ir_value_mm)
        maximum_ir_value_mm = max(self._lower_ir_value_mm, self._upper_ir_value_mm)
        if ir_value_mm < minimum_ir_value_mm or ir_value_mm > maximum_ir_value_mm:
            self._update_logger.warning(
                f'Update #{update_id}: ir_value ({ir_value_mm} mm) is outside of expected range '
                f'[{minimum_ir_value_mm} mm, {maximum_ir_value_mm} mm]'
            )
            return

        # Transform the sensor value read from the serial port (in mm) to a position (in mm) using
        # a linear mapping based on the configured limits and IR value range.
        # This determines the distance to the target and the required motor command.
        current_pos_mm = (
            self._ir_value_to_position_slope * (ir_value_mm - self._lower_ir_value_mm)
            + self._lower_limit_mm
        )

        if self._filtered_current_pos_mm is None:
            self._filtered_current_pos_mm = current_pos_mm
        else:
            alpha = self._low_pass_filter_coeff
            self._filtered_current_pos_mm = (
                alpha * current_pos_mm + (1.0 - alpha) * self._filtered_current_pos_mm
            )

        # Publish the measured joint position before any command-processing early returns.
        # This keeps the JointState stream alive even when no target command has been received.
        self._publish_joint_state(self._filtered_current_pos_mm)

        # If the target position is not set, we cannot determine the target sensor value or the
        # command, so we skip the update.
        if self._target_pos_mm is None:
            return

        # This guard protects against internal state corruption. The topic callback rejects invalid
        # targets before storing them.
        if (
            self._target_pos_mm < self._lower_limit_mm
            or self._target_pos_mm > self._upper_limit_mm
        ):
            self._serial_update_logger.warning(
                f'Update #{update_id}: Target position '
                f'({self._target_pos_mm / 1000.0} m) is outside '
                f'[{self._lower_limit_mm / 1000.0} m, {self._upper_limit_mm / 1000.0} m]'
            )
            return

        # If the target position is valid and has already been reached, we can skip the update
        # without sending any command.
        if self._target_position_reached:
            return

        # 0: stop
        # 1: move up
        # 2: move down
        # Compute the command based on the difference between the target sensor value and the
        # current sensor value,
        dist = self._target_pos_mm - self._filtered_current_pos_mm
        cmd = None

        if abs(dist) <= self._convergence_threshold_mm:
            cmd = 0
        elif dist > 0:
            cmd = 1
        else:
            cmd = 2

        # Send a command only when it differs from the motor status reported by the firmware.
        if str(cmd) == motor_status:
            if cmd == 0:
                self._target_position_reached = True
            return

        # The Arduino sketch reads one byte with `Serial.read()`. Do not append a line ending
        # here, because `\n` or `\r` would be consumed as a second command byte.
        try:
            written_bytes = self._serial.write(f'{cmd}'.encode('ascii'))
        except serial.SerialException as exc:
            self._update_logger.error(f'Failed to update serial port: {exc}')
            return

        if written_bytes != 1:
            self._update_logger.error(
                f'Failed to update serial port: wrote {written_bytes} of 1 command byte.'
            )
            return

        # Mark the target as reached only after the stop command was accepted by the serial port.
        # An earlier write failure leaves this flag false, so the next update retries the command.
        if cmd == 0:
            self._target_position_reached = True

    def _publish_joint_state(self, current_pos_mm: float | None = None) -> None:
        if current_pos_mm is None or self._joint_state_publisher is None:
            return

        current_pos_m = current_pos_mm / 1000.0

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [self._joint_name]
        joint_state_msg.position = [current_pos_m]
        self._joint_state_publisher.publish(joint_state_msg)

    def _read(self) -> tuple[str, int] | None:
        if self._serial is None or not self._serial.is_open:
            return None

        try:
            waiting_bytes = self._serial.in_waiting

            if waiting_bytes <= 0:
                return None

            self._serial_read_buffer += self._serial.read(waiting_bytes)

            if b'\n' not in self._serial_read_buffer:
                return None

            lines = self._serial_read_buffer.split(b'\n')
            self._serial_read_buffer = lines[-1]
            latest_read: tuple[str, int] | None = None

            for raw_line in lines[:-1]:
                line = raw_line.decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                try:
                    status_text, sensor_value_text = line.rsplit(',', maxsplit=1)
                    latest_read = (status_text.strip(), int(sensor_value_text.strip()))
                except ValueError:
                    self._update_logger.warning(f'Ignoring invalid serial payload: {line!r}')
            return latest_read
        except serial.SerialException as exc:
            self._update_logger.error(f'Failed to read sensor value from serial port: {exc}')
            return None
