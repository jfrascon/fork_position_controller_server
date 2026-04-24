from __future__ import annotations

import time
from typing import Final

import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class ForkSerialNode(Node):
    """
    Class to interface with the serial connection of the fork.
    The class reads the sensor value from the serial connection and publishes it to a ROS topic.
    It also subscribes to a ROS topic for the target position and sends commands to the serial
    connection to move the fork accordingly.
    """

    FLOW_CONTROL_NONE: Final[str] = 'none'
    FLOW_CONTROL_HARDWARE: Final[str] = 'hardware'
    FLOW_CONTROL_SOFTWARE: Final[str] = 'software'
    VALID_FLOW_CONTROLS: Final[set[str]] = {FLOW_CONTROL_NONE, FLOW_CONTROL_HARDWARE, FLOW_CONTROL_SOFTWARE}

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
        super().__init__('fork_serial')

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
        self.declare_parameter('convergence_threshold', 0.03)
        self.declare_parameter('update_frequency', 2.0)
        self.declare_parameter('low_pass_filter_coeff', 0.10)
        self.declare_parameter('joint_name', '')
        self.declare_parameter('command_topic', 'fork_command')
        self.declare_parameter('joint_states_topic', 'joint_states')
        # self.declare_parameter('position_topic', 'fork_position')
        # self.declare_parameter('sensor_value_topic', 'fork_sensor_value')

        # ROS encourages using SI units, so we usually use meters, rads, etc. However, the
        # serial communication uses millimeters for the position, so we need to convert the limits
        # and the convergence threshold from meters to millimeters.
        self._lower_limit_mm = self.get_parameter('lower_limit').get_parameter_value().double_value * 1000.0
        self._upper_limit_mm = self.get_parameter('upper_limit').get_parameter_value().double_value * 1000.0
        self._lower_ir_value_mm = self.get_parameter('lower_ir_value').get_parameter_value().double_value * 1000.0
        self._upper_ir_value_mm = self.get_parameter('upper_ir_value').get_parameter_value().double_value * 1000.0

        # Maintain a value in meters and millimeters, since at different moments we need to use one or the other, so
        # to avoid doing repeated conversions we keep both values as member variables.
        self._convergence_threshold = self.get_parameter('convergence_threshold').get_parameter_value().double_value
        self._convergence_threshold_mm = self._convergence_threshold * 1000.0

        if self._lower_limit_mm == self._upper_limit_mm:
            raise ValueError('Parameters lower_limit and upper_limit must be different.')

        if self._lower_ir_value_mm == self._upper_ir_value_mm:
            raise ValueError('Parameters lower_ir_value and upper_ir_value must be different.')

        position_range_mm = self._upper_limit_mm - self._lower_limit_mm
        ir_range_mm = self._upper_ir_value_mm - self._lower_ir_value_mm

        # Slope to convert from position to IR value: ir_value = slope * (position - lower_limit) + lower_ir_value
        self._position_to_ir_value_slope = ir_range_mm / position_range_mm

        # Slope to convert from IR value to position: position = slope * (ir_value - lower_ir_value) + lower_limit
        self._ir_value_to_position_slope = position_range_mm / ir_range_mm

        # Like the convergence threshold, we maintain the target position in both millimeters and meters to avoid
        # repeated conversions.
        self._target_pos_mm: float | None = None
        self._target_position: float | None = None

        self._target_position_reached: bool = False
        self._target_ir_value_mm: int | None = None
        self._previous_target_sensor_value: float | None = None
        self._serial: serial.Serial | None = None
        self._serial_read_buffer = b''
        self._joint_state_publisher = None
        self._joint_name = ''
        self._filtered_current_pos_mm: float | None = None
        self._low_pass_filter_coeff = self.get_parameter('low_pass_filter_coeff').get_parameter_value().double_value
        self._update_logger = self.get_logger().get_child('update')
        self._serial_update_logger = self.get_logger().get_child('serial_update')
        self._target_position_logger = self.get_logger().get_child('target_position')
        self._update_sequence = 0
        self._update_period_s = 0.0

        self._open_serial_port()

        joint_name = self.get_parameter('joint_name').get_parameter_value().string_value.strip()
        command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        # position_topic = self.get_parameter('position_topic').get_parameter_value().string_value
        # sensor_value_topic = self.get_parameter('sensor_value_topic').get_parameter_value().string_value
        joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value.strip()

        if not joint_name:
            raise ValueError("Parameter 'joint_name' must not be empty.")

        self._joint_name = joint_name
        self._subscriber = self.create_subscription(Float64, command_topic, self._target_pos_callback, 10)
        self._joint_state_publisher = self.create_publisher(JointState, joint_states_topic, 10)

        # self._position_publisher = self.create_publisher(Float64, position_topic, 10)
        # self._sensor_value_publisher = self.create_publisher(Int32, sensor_value_topic, 10)

        # Frecuency at which to update the serial connection and publish the sensor value and position.
        update_frequency = self.get_parameter('update_frequency').get_parameter_value().double_value

        if update_frequency <= 0.0:
            raise ValueError('Parameter update_frequency must be greater than 0.0 Hz.')

        if self._low_pass_filter_coeff < 0.0 or self._low_pass_filter_coeff > 1.0:
            raise ValueError("Parameter 'low_pass_filter_coeff' must be in the inclusive range [0.0, 1.0].")

        self._update_period_s = 1.0 / update_frequency
        self._timer = self.create_timer(self._update_period_s, self._update)

        self.get_logger().get_child('init').info(
            f"Updating fork serial connection on '{self._serial.port}' at '{self._serial.baudrate}' baud"
        )

    def destroy_node(self) -> bool:
        timer = getattr(self, '_timer', None)
        if timer is not None:
            timer.cancel()

        serial_port = getattr(self, '_serial', None)
        if serial_port is not None and serial_port.is_open:
            serial_port.close()

        return super().destroy_node()

    def _target_pos_callback(self, msg: Float64) -> None:
        if self._target_position is not None:
            dist = abs(msg.data - self._target_position)

            if dist <= self._convergence_threshold:
                # self._target_position_logger.warning(f'Ignoring received position: {msg.data} m ({dist} m)')
                return

        self._target_position = msg.data
        self._target_pos_mm = self._target_position * 1000.0
        self._target_position_reached = False
        # self._target_position_logger.info(f"Received position: '{self._target_pos_mm}' mm")

    def _open_serial_port(self) -> None:
        self._close_serial_port()

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        data_bits = self.get_parameter('data_bits').get_parameter_value().integer_value
        flow_control = self.get_parameter('flow_control').get_parameter_value().string_value.lower()
        parity = self.get_parameter('parity').get_parameter_value().string_value.lower()
        stop_bits = self.get_parameter('stop_bits').get_parameter_value().double_value

        self._validate_serial_configuration(data_bits, flow_control, parity, stop_bits)

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
            self._serial.close()
        self._serial = None

    def _validate_serial_configuration(self, data_bits: int, flow_control: str, parity: str, stop_bits: float) -> None:
        if data_bits not in self.VALID_BYTE_SIZES:
            raise ValueError(f'Unsupported data_bits value: {data_bits}. Expected one of 5, 6, 7, 8.')

        if flow_control not in self.VALID_FLOW_CONTROLS:
            raise ValueError(
                f"Unsupported flow_control value: '{flow_control}'. Expected 'none', 'hardware' or 'software'."
            )

        if parity not in self.VALID_PARITIES:
            raise ValueError(
                f"Unsupported parity value: '{parity}'. Expected 'none', 'even', 'odd', 'mark' or 'space'."
            )

        if stop_bits not in self.VALID_STOP_BITS:
            raise ValueError(f"Unsupported stop_bits value: '{stop_bits}'. Expected 1, 1.5 or 2.")

    def _update(self) -> None:
        self._update_sequence += 1
        update_id = self._update_sequence
        # update_start = time.perf_counter()

        # self._update_logger.info(f'Update #{update_id}: Started')

        if self._serial is None or not self._serial.is_open:
            self._update_logger.error(f'Update #{update_id}: Serial port is not open. Skipping serial update.')
            return

        # read_start = time.perf_counter()
        read_result = self._read(update_id)
        # read_duration_ms = (time.perf_counter() - read_start) * 1000.0

        # No valid data read from the serial port, skipping update.
        if read_result is None:
            # self._update_logger.info(
            #     f'Update #{update_id}: Finished without serial frame after {read_duration_ms:.3f} ms'
            # )
            # self._log_update_duration(update_id, update_start)
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
                f'Update #{update_id}: Ignored invalid frame.'
                'status = {motor_status!r}, ir_value = {ir_value_mm!r} mm'
            )
            # self._log_update_duration(update_id, update_start)
            return

        if ir_value_mm < self._lower_ir_value_mm or ir_value_mm > self._upper_ir_value_mm:
            self._update_logger.warning(
                f'Update #{update_id}: ir_value ({ir_value_mm} mm) is outside of expected range '
                f'[{self._lower_ir_value_mm} mm, {self._upper_ir_value_mm} mm]'
            )
            # self._log_update_duration(update_id, update_start)
            return

        # self._update_logger.info(
        #     f'Update #{update_id}: status = {motor_status}, ir_value: {ir_value_mm} mm, '
        #     f'read_duration = {read_duration_ms:.3f} ms'
        # )

        # Transform the sensor value read from the serial port (in mm) to a position (in mm) using
        # a linear mapping based on the configured limits and IR value range.
        # This allows us to determine how far the fork is from the target position and which command
        # to send to the serial port.
        current_pos_mm = (
            self._ir_value_to_position_slope * (ir_value_mm - self._lower_ir_value_mm) + self._lower_limit_mm
        )

        if self._filtered_current_pos_mm is None:
            self._filtered_current_pos_mm = current_pos_mm
        else:
            alpha = self._low_pass_filter_coeff
            self._filtered_current_pos_mm = alpha * current_pos_mm + (1.0 - alpha) * self._filtered_current_pos_mm

        # Publish the measured fork position before any command-processing early returns.
        # This keeps the JointState stream alive even when no target command has been received.
        self._publish_joint_state(self._filtered_current_pos_mm)

        # self._publish_position(ir_value_mm)
        # self._publish_sensor_value(ir_value_mm)

        # If the target position is not set, we cannot determine the target sensor value or the
        # command, so we skip the update.
        if self._target_pos_mm is None:
            # self._update_logger.info(f'Update #{update_id}: No target position yet')
            # self._log_update_duration(update_id, update_start)
            return

        # If the target position is outside the configured limits, we log a warning and finish the update without
        # sending any command.
        if self._target_pos_mm < self._lower_limit_mm or self._target_pos_mm > self._upper_limit_mm:
            self._serial_update_logger.warning(
                f'Update #{update_id}: Target position ({self._target_pos_mm / 1000.0} mm) is outside '
                f'[{self._lower_limit_mm / 1000.0} mm, {self._upper_limit_mm / 1000.0} mm]'
            )
            # self._log_update_duration(update_id, update_start)
            return

        # If the target position is valid and has already been reached, we can skip the update
        # without sending any command.
        if self._target_position_reached:
            # self._update_logger.info(f'Update #{update_id}: TARGET POSITION ALREADY REACHED')
            # self._log_update_duration(update_id, update_start)
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
            self._target_position_reached = True
            # self._update_logger.info(f'Update #{update_id}: TARGET POSITION REACHED')
        elif dist > 0:
            cmd = 1
        else:
            cmd = 2

        # self._serial_update_logger.info(
        #     f'Update #{update_id}: target_pos = {self._target_pos_mm} mm, '
        #     f'current_pos = {current_pos_mm} mm, '
        #     f'dist = {dist} mm, '
        #     f'convergence reached = {str(self._target_position_reached).upper()}, '
        #     f'cmd = {cmd}, '
        #     f'status = {motor_status}, '
        #     f'ir_value = {ir_value_mm} mm, '
        # )

        # Send the command to the serial port only if the command is different from the current
        # motor status.
        # This prevents unnecessary serial writes when the fork is already

        # The Arduino sketch reads one byte with `Serial.read()`. Do not append a line ending
        # here, because `\n` or `\r` would be consumed as a second command byte.
        if str(cmd) != motor_status:
            try:
                # self._serial_update_logger.info(f"Update #{update_id}: Writing serial command '{cmd}'")
                # write_start = time.perf_counter()
                self._serial.write(f'{cmd}'.encode('ascii'))

                # write_duration_ms = (time.perf_counter() - write_start) * 1000.0
                # self._serial_update_logger.info(
                #     f"Update #{update_id}: Wrote serial command '{cmd}' in {write_duration_ms:.3f} ms"
                # )
            except serial.SerialException as exc:
                self._update_logger.error(f'Failed to update serial port: {exc}')
        # else:
        #     self._serial_update_logger.info(
        #         f"Update #{update_id}: Skipped serial write because command '{cmd}' equals status {motor_status}"
        #     )

        # self._log_update_duration(update_id, update_start)

    def _publish_joint_state(self, current_pos_mm: float | None = None) -> None:
        if current_pos_mm is None or self._joint_state_publisher is None:
            return

        current_pos_m = current_pos_mm / 1000.0

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [self._joint_name]
        joint_state_msg.position = [current_pos_m]
        self._joint_state_publisher.publish(joint_state_msg)

    # def _publish_position(self, ir_value_mm: int | None = None) -> None:
    #     if ir_value_mm is None:
    #         return

    #     position_msg = Float64()
    #     dist = ir_value_mm - self._lower_ir_value_mm
    #     position_msg.data = self._ir_value_to_position_slope * dist + self._lower_limit_mm
    #     self._position_publisher.publish(position_msg)

    # def _publish_sensor_value(self, ir_value_mm: int | None = None) -> None:
    #     if ir_value_mm is None:
    #         return

    #     sensor_value_msg = Int32()
    #     sensor_value_msg.data = ir_value_mm
    #     self._sensor_value_publisher.publish(sensor_value_msg)

    def _log_update_duration(self, update_id: int, update_start: float) -> None:
        update_duration_s = time.perf_counter() - update_start
        update_duration_ms = update_duration_s * 1000.0

        if self._update_period_s > 0.0 and update_duration_s > self._update_period_s:
            self._update_logger.warning(
                f'Update #{update_id}: Took {update_duration_ms:.3f} ms, which is longer than the '
                f'configured period of {self._update_period_s * 1000.0:.3f} ms.'
            )
            return

        self._update_logger.info(f'Update #{update_id}: Finished in {update_duration_ms:.3f} ms\n')

    def _read(self, update_id: int | None = None) -> tuple[str, int] | None:
        if self._serial is None or not self._serial.is_open:
            return None

        try:
            waiting_bytes = self._serial.in_waiting

            if waiting_bytes <= 0:
                # if update_id is not None:
                #     self._update_logger.info(f'Update #{update_id}: No serial bytes waiting')
                return None

            self._serial_read_buffer += self._serial.read(waiting_bytes)

            if b'\n' not in self._serial_read_buffer:
                # if update_id is not None:
                #     self._update_logger.info(
                #         f'Update #{update_id}: There are {len(self._serial_read_buffer)} buffered bytes but no newline yet'
                #     )
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
            # if update_id is not None:
            #     self._update_logger.info(
            #         f'Update #{update_id}: Parsed serial frame {latest_read!r} from {len(lines) - 1} line(s)'
            #     )
            return latest_read
        except serial.SerialException as exc:
            self._update_logger.error(f'Failed to read sensor value from serial port: {exc}')
            return None
