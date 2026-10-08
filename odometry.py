from pybricks.tools import DataLog, StopWatch


class OdometryLogger:
    def __init__(self, robot, left_motor, right_motor, line_sensor,
                 touch_sensor, gyro_sensor, interval_ms=100):
        self.robot = robot
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.line_sensor = line_sensor
        self.touch_sensor = touch_sensor
        self.gyro_sensor = gyro_sensor
        self.interval_ms = interval_ms
        self.stopwatch = StopWatch()
        self.last_log_time = -interval_ms
        self.data_log = DataLog(
            "time_ms",
            "phase",
            "segment_distance_mm",
            "distance_reference_mm",
            "drivebase_angle_deg",
            "gyro_angle_deg",
            "gyro_reference_deg",
            "gyro_error_deg",
            "drive_speed_command_mm_s",
            "turn_rate_command_deg_s",
            "left_motor_angle_deg",
            "left_motor_speed_deg_s",
            "right_motor_angle_deg",
            "right_motor_speed_deg_s",
            "color_reflection",
            "button_pressed",
            name="square_odometry",
            timestamp=True,
            extension="csv",
        )

    def record(self, phase, distance_reference, heading_reference,
               drive_speed, turn_rate):
        current_time = self.stopwatch.time()
        if current_time - self.last_log_time < self.interval_ms:
            return

        gyro_angle = self.gyro_sensor.angle()
        self.data_log.log(
            current_time,
            phase,
            self.robot.distance(),
            distance_reference,
            self.robot.angle(),
            gyro_angle,
            heading_reference,
            heading_reference - gyro_angle,
            drive_speed,
            turn_rate,
            self.left_motor.angle(),
            self.left_motor.speed(),
            self.right_motor.angle(),
            self.right_motor.speed(),
            self.line_sensor.reflection(),
            self.touch_sensor.pressed(),
        )
        self.last_log_time = current_time
