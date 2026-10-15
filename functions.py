from pybricks.tools import wait

# Intervalo de amostragem da malha de controle. Usado no wait() dos lacos de
# movimento (ms) e nos termos integral e derivativo do PID (s).
SAMPLE_TIME_MS = 10
SAMPLE_TIME = SAMPLE_TIME_MS / 1000

# Velocidade minima de aproximacao no fim de um trecho reto, em mm/s.
MIN_APPROACH_SPEED = 60

# Velocidade usada enquanto o robo procura a faixa preta de chegada, em mm/s.
# Mais baixa que a de cruzeiro para o robo parar logo depois de detectar a
# faixa, em vez de passar por cima dela.
LINE_SEARCH_SPEED = 100

# Fracao do ultimo trecho percorrida antes de comecar a procurar a faixa. Evita
# parar em alguma marca escura no comeco do trecho.
LINE_SEARCH_START = 0.5


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


def drive_to_line(robot, gyro_sensor, pid_params, odometry, line_sensor,
                  black_threshold, distance, drive_speed):
    """Anda em linha reta, mantendo o heading atual com o PID do giroscopio,
    ate o sensor de cor encontrar a faixa preta.

    O sensor fica a frente do eixo das rodas, entao depois da meia volta ele
    chega na faixa antes do eixo voltar ao ponto de partida. Parar pela faixa
    corrige esse deslocamento e tambem o erro acumulado da odometria.

    A faixa so e procurada depois de LINE_SEARCH_START * 'distance' mm. Se ela
    nao aparecer ate 'distance' mm, o robo para ali, como em drive_straight."""
    pid_params.reference = gyro_sensor.angle()
    pid_params.last_error = 0
    pid_params.ie_dt = 0
    robot.reset()

    search_start = distance * LINE_SEARCH_START

    while robot.distance() < distance:
        if robot.distance() < search_start:
            speed = drive_speed
        else:
            if line_sensor.reflection() <= black_threshold:
                break
            speed = LINE_SEARCH_SPEED
        turn_rate = gyro_pid(robot, gyro_sensor, speed, pid_params)
        odometry.record("to_line", distance, pid_params.reference, speed,
                        turn_rate)
        wait(SAMPLE_TIME_MS)

    robot.stop()


def run_square(robot, gyro_sensor, pid_params, odometry, segments, direction,
               drive_speed, line_sensor=None, black_threshold=None):
    """Percorre o quadrado descrito por 'segments', uma sequencia de
    (angulo_de_giro, distancia_mm).

    'direction' inverte o sinal dos giros e com isso o sentido do percurso:
    1 para o sentido horario e -1 para o anti-horario. Inverter o sinal e o
    que realmente espelha o caminho - apenas dar meia volta antes de repetir a
    mesma sequencia de giros manteria o percurso no sentido original.

    Se 'line_sensor' for informado, o ultimo trecho termina na faixa preta
    (drive_to_line) em vez de pela distancia."""
    last_index = len(segments) - 1
    for index, (turn_angle, distance) in enumerate(segments):
        if turn_angle is not None:
            turn_in_place(robot, gyro_sensor, pid_params, odometry,
                          turn_angle * direction)
        if distance > 0:
            if line_sensor is not None and index == last_index:
                drive_to_line(robot, gyro_sensor, pid_params, odometry,
                              line_sensor, black_threshold, distance,
                              drive_speed)
            else:
                drive_straight(robot, gyro_sensor, pid_params, odometry,
                               distance, drive_speed)
