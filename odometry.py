from math import cos, degrees, pi, radians, sin

from pybricks.tools import DataLog, StopWatch


class OdometryLogger:
    """Registra em CSV, em tempo real, tudo o que os sensores do robo medem
    e a odometria calculada a partir dessas medidas.

    Separacao usada nas colunas:

    - medido: vem direto de um sensor, na unidade do proprio sensor
      (encoders em graus e graus/s, giroscopio em graus e graus/s,
      sensor de cor em % de reflexao, botao em booleano);
    - calculado: nao existe sensor que meca a grandeza diretamente, entao
      ela e obtida das medidas + geometria do robo (distancia percorrida,
      heading pelas rodas e posicao x/y por dead-reckoning);
    - referencia / comando: setpoints de distancia e angulo e a variavel
      manipulada do PID (velocidade e taxa de giro enviadas ao DriveBase).

    A posicao e integrada por dois caminhos independentes - um usando o
    heading medido pelo giroscopio e outro usando o heading calculado pelos
    encoders - justamente para permitir a comparacao calculado vs medido.
    """

    def __init__(self, robot, left_motor, right_motor, line_sensor,
                 touch_sensor, gyro_sensor, wheel_diameter, axle_track,
                 interval_ms=100):
        self.robot = robot
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.line_sensor = line_sensor
        self.touch_sensor = touch_sensor
        self.gyro_sensor = gyro_sensor
        self.wheel_diameter = wheel_diameter
        self.axle_track = axle_track
        self.interval_ms = interval_ms

        self.stopwatch = StopWatch()
        self.last_log_time = -interval_ms

        # Estado da integracao da pose. Comeca em zero porque os encoders e o
        # giroscopio sao zerados antes do inicio do percurso.
        self.last_distance = 0
        self.x_gyro = 0
        self.y_gyro = 0
        self.x_wheel = 0
        self.y_wheel = 0

        self.data_log = DataLog(
            "time_ms",
            "phase",

            # Referencias de movimentacao guiada e variavel manipulada.
            "distance_reference_mm",
            "heading_reference_deg",
            "heading_error_deg",
            "drive_speed_command_mm_s",
            "turn_rate_command_deg_s",

            # Medidas diretas dos sensores.
            "gyro_angle_deg",
            "left_motor_angle_deg",
            "left_motor_speed_deg_s",
            "right_motor_angle_deg",
            "right_motor_speed_deg_s",
            "line_reflection_pct",
            "button_pressed",

            # Odometria calculada a partir das medidas.
            "segment_distance_mm",
            "wheel_distance_mm",
            "wheel_heading_deg",
            "heading_diff_deg",
            "x_gyro_mm",
            "y_gyro_mm",
            "x_wheel_mm",
            "y_wheel_mm",

            name="square_odometry",
            timestamp=True,
            extension="csv",
        )

    def wheel_travel(self, motor_angle):
        # Converte o angulo medido pelo encoder (graus) em deslocamento
        # linear da roda (mm).
        return motor_angle * pi * self.wheel_diameter / 360

    def wheel_distance(self, left_travel, right_travel):
        return (left_travel + right_travel) / 2

    def wheel_heading(self, left_travel, right_travel):
        # Cinematica do acionamento diferencial: a diferenca entre o
        # deslocamento das duas rodas dividida pela bitola da o giro do robo
        # em radianos. A roda esquerda andar mais que a direita significa giro
        # no sentido horario, que e o sentido positivo adotado aqui.
        return degrees((left_travel - right_travel) / self.axle_track)

    def update_pose(self, distance, gyro_angle, wheel_heading):
        # Nenhum sensor do robo mede posicao, entao x e y sao integrados a
        # cada amostra: o quanto o robo andou desde a amostra anterior,
        # projetado no heading daquele instante (dead-reckoning).
        step = distance - self.last_distance
        self.last_distance = distance

        # O heading cresce no sentido horario, enquanto o plano x/y segue a
        # convencao matematica (anti-horario positivo). O sinal negativo faz o
        # grafico da trajetoria sair com o mesmo formato do percurso real.
        gyro_rad = radians(-gyro_angle)
        wheel_rad = radians(-wheel_heading)

        self.x_gyro += step * cos(gyro_rad)
        self.y_gyro += step * sin(gyro_rad)
        self.x_wheel += step * cos(wheel_rad)
        self.y_wheel += step * sin(wheel_rad)

    def record(self, phase, distance_reference, heading_reference,
               drive_speed, turn_rate):
        current_time = self.stopwatch.time()
        if current_time - self.last_log_time < self.interval_ms:
            return

        # Cada medida e lida uma unica vez por amostra, para que o valor
        # registrado e o valor usado nos calculos sejam o mesmo.
        gyro_angle = self.gyro_sensor.angle()
        left_angle = self.left_motor.angle()
        right_angle = self.right_motor.angle()

        left_travel = self.wheel_travel(left_angle)
        right_travel = self.wheel_travel(right_angle)
        wheel_distance = self.wheel_distance(left_travel, right_travel)
        wheel_heading = self.wheel_heading(left_travel, right_travel)

        self.update_pose(wheel_distance, gyro_angle, wheel_heading)

        self.data_log.log(
            current_time,
            phase,

            distance_reference,
            heading_reference,
            heading_reference - gyro_angle,
            drive_speed,
            turn_rate,

            gyro_angle,
            left_angle,
            self.left_motor.speed(),
            right_angle,
            self.right_motor.speed(),
            self.line_sensor.reflection(),
            self.touch_sensor.pressed(),

            self.robot.distance(),
            wheel_distance,
            wheel_heading,
            wheel_heading - gyro_angle,
            self.x_gyro,
            self.y_gyro,
            self.x_wheel,
            self.y_wheel,
        )
        self.last_log_time = current_time
