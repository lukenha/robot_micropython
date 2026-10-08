from pybricks.tools import wait

# Intervalo de amostragem da malha de controle. Usado no wait() dos lacos de
# movimento (ms) e nos termos integral e derivativo do PID (s).
SAMPLE_TIME_MS = 10
SAMPLE_TIME = SAMPLE_TIME_MS / 1000

# Velocidade minima de aproximacao no fim de um trecho reto, em mm/s.
MIN_APPROACH_SPEED = 60


def gyro_pid(robot, gyro_sensor, drive_speed, pid_params):
    """Calcula e aplica uma correcao de direcao para manter o heading medido
    pelo giroscopio igual a referencia. Retorna a variavel manipulada
    (taxa de giro, em graus/s)."""
    error = pid_params.reference - gyro_sensor.angle()

    integral = pid_params.ie_dt + error * SAMPLE_TIME
    if integral > 50:
        integral = 50
    elif integral < -50:
        integral = -50

    derivative = (error - pid_params.last_error) / SAMPLE_TIME

    turn_rate = (pid_params.proportional_gain * error
                 + pid_params.integrative_gain * integral
                 + pid_params.derivative_gain * derivative)

    if turn_rate > 100:
        turn_rate = 100
    elif turn_rate < -100:
        turn_rate = -100
    elif drive_speed == 0 and abs(error) > 1 and abs(turn_rate) < 15:
        # Girando no proprio eixo, um comando muito fraco nao vence o atrito e
        # o robo trava antes de chegar ao alvo.
        turn_rate = 15 if error > 0 else -15

    robot.drive(drive_speed, turn_rate)

    pid_params.last_error = error
    pid_params.ie_dt = integral
    return turn_rate


def turn_in_place(robot, gyro_sensor, pid_params, odometry, angle):
    """Gira o robo no proprio eixo ate o angulo do giroscopio variar 'angle'
    graus. Angulo positivo gira no sentido horario, negativo no anti-horario."""
    pid_params.reference = gyro_sensor.angle() + angle
    pid_params.last_error = angle
    pid_params.ie_dt = 0

    while abs(pid_params.reference - gyro_sensor.angle()) > 1:
        turn_rate = gyro_pid(robot, gyro_sensor, 0, pid_params)
        odometry.record("turn", 0, pid_params.reference, 0, turn_rate)
        wait(SAMPLE_TIME_MS)

    robot.stop()
    wait(100)


def drive_straight(robot, gyro_sensor, pid_params, odometry, distance,
                   drive_speed):
    """Anda 'distance' mm em linha reta, mantendo o heading atual com o PID do
    giroscopio e desacelerando no fim do trecho para nao passar do alvo."""
    pid_params.reference = gyro_sensor.angle()
    pid_params.last_error = 0
    pid_params.ie_dt = 0
    robot.reset()

    while robot.distance() < distance:
        remaining = distance - robot.distance()
        speed = min(drive_speed, max(MIN_APPROACH_SPEED, remaining * 2))
        turn_rate = gyro_pid(robot, gyro_sensor, speed, pid_params)
        odometry.record("straight", distance, pid_params.reference, speed,
                        turn_rate)
        wait(SAMPLE_TIME_MS)

    robot.stop()


def run_square(robot, gyro_sensor, pid_params, odometry, segments, direction,
               drive_speed):
    """Percorre o quadrado descrito por 'segments', uma sequencia de
    (angulo_de_giro, distancia_mm).

    'direction' inverte o sinal dos giros e com isso o sentido do percurso:
    1 para o sentido horario e -1 para o anti-horario. Inverter o sinal e o
    que realmente espelha o caminho - apenas dar meia volta antes de repetir a
    mesma sequencia de giros manteria o percurso no sentido original."""
    for turn_angle, distance in segments:
        if turn_angle is not None:
            turn_in_place(robot, gyro_sensor, pid_params, odometry,
                          turn_angle * direction)
        if distance > 0:
            drive_straight(robot, gyro_sensor, pid_params, odometry, distance,
                           drive_speed)
