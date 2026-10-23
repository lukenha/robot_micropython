from pybricks.tools import wait

# Intervalo de amostragem da malha de controle. Usado no wait() dos lacos de
# movimento (ms) e nos termos integral e derivativo do PID (s).
SAMPLE_TIME_MS = 10
SAMPLE_TIME = SAMPLE_TIME_MS / 1000

# Velocidade minima de aproximacao no fim de um trecho reto, em mm/s.
MIN_APPROACH_SPEED = 60

# Torre do ultrassom: velocidade (graus/s) e angulos absolutos do motor para
# olhar para cada lado. O angulo 0 e o sensor apontado para a frente. O sinal
# depende da montagem: no teste com o robo, -90 olhava para a direita, entao
# a esquerda e +90 e a direita e -90.
TURRET_SPEED = 500
TURRET_LEFT = 90
TURRET_RIGHT = -90

# Tempo para o ultrassom estabilizar depois que a torre para, em ms.
TURRET_SETTLE_MS = 100


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


def turn_in_place(robot, gyro_sensor, pid_params, odometry, heading):
    """Gira o robo no proprio eixo ate o giroscopio marcar o heading absoluto
    'heading' (graus, multiplo de 90 contado desde a largada). Usar o heading
    absoluto, e nao o angulo atual + giro, evita que o erro de cada curva se
    acumule nas seguintes. Heading maior que o atual gira no sentido horario,
    menor gira no anti-horario."""
    pid_params.reference = heading
    pid_params.last_error = heading - gyro_sensor.angle()
    pid_params.ie_dt = 0

    while abs(pid_params.reference - gyro_sensor.angle()) > 1:
        turn_rate = gyro_pid(robot, gyro_sensor, 0, pid_params)
        odometry.record("turn", 0, pid_params.reference, 0, turn_rate)
        wait(SAMPLE_TIME_MS)

    robot.stop()
    wait(100)


def drive_until_wall(robot, gyro_sensor, pid_params, odometry, heading,
                     ultrasonic_sensor,
                     # line_sensor, black_threshold,
                     stop_distance, drive_speed):
    """Anda em linha reta, mantendo o heading absoluto 'heading' com o PID do
    giroscopio, ate o ultrassom medir uma parede a 'stop_distance' mm ou o
    sensor de cor encontrar a faixa preta de chegada. Desacelera ao se
    aproximar da parede. Se a curva anterior parou um pouco antes ou depois do
    alvo, o PID corrige essa diferenca durante a reta.

    Retorna True se parou na faixa preta e False se parou na parede.

    O robo larga em cima da faixa preta, entao a faixa so passa a contar depois
    que o sensor de cor ve o chao claro pela primeira vez no trecho.

    SENSOR DE LINHA DESATIVADO: a deteccao da faixa preta esta comentada
    enquanto o sensor de cor nao esta montado, entao a funcao so para na
    parede e sempre retorna False."""
    pid_params.reference = heading
    pid_params.last_error = heading - gyro_sensor.angle()
    pid_params.ie_dt = 0
    robot.reset()

    # seen_floor = False

    while True:
        # if line_sensor.reflection() <= black_threshold:
        #     if seen_floor:
        #         robot.stop()
        #         return True
        # else:
        #     seen_floor = True

        # Sem obstaculo o ultrassom mede 2550 mm, que conta como caminho livre.
        front = ultrasonic_sensor.distance()
        if front <= stop_distance:
            robot.stop()
            return False

        speed = min(drive_speed,
                    max(MIN_APPROACH_SPEED, (front - stop_distance) * 2))
        turn_rate = gyro_pid(robot, gyro_sensor, speed, pid_params)
        odometry.record("maze", stop_distance, pid_params.reference, speed,
                        turn_rate)
        wait(SAMPLE_TIME_MS)


def scan_sides(turret_motor, ultrasonic_sensor):
    """Gira a torre do ultrassom para a esquerda e para a direita, mede a
    distancia ate a parede em cada lado e volta a apontar para a frente.
    Retorna (esquerda, direita) em mm."""
    turret_motor.run_target(TURRET_SPEED, TURRET_LEFT)
    wait(TURRET_SETTLE_MS)
    left = ultrasonic_sensor.distance()

    turret_motor.run_target(TURRET_SPEED, TURRET_RIGHT)
    wait(TURRET_SETTLE_MS)
    right = ultrasonic_sensor.distance()

    turret_motor.run_target(TURRET_SPEED, 0)
    return left, right
