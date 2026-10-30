from pybricks.tools import StopWatch, wait

# Intervalo de amostragem da malha de controle. Usado no wait() dos lacos de
# movimento (ms) e nos termos integral e derivativo do PID (s).
SAMPLE_TIME_MS = 10
SAMPLE_TIME = SAMPLE_TIME_MS / 1000

# Desaceleracao no fim de cada avanco: a velocidade e a distancia restante
# vezes APPROACH_GAIN (mm/s por mm), sem cair abaixo de MIN_APPROACH_SPEED.
# Com ganho 3, o robo so comeca a frear a ~65 mm do fim do trecho. Se ele
# passar do ponto, reduzir o ganho ou a velocidade minima.
APPROACH_GAIN = 3
MIN_APPROACH_SPEED = 70

# Espera depois de parar um giro ou um alinhamento, para o robo assentar
# antes da proxima leitura, em ms.
SETTLE_AFTER_MOVE_MS = 50

# Alinhamento pela parede da frente: tolerancia (mm), ganho (mm/s por mm de
# erro), limites de velocidade (mm/s) e tempo maximo da manobra (ms).
ALIGN_TOLERANCE = 5
ALIGN_GAIN = 3
ALIGN_MIN_SPEED = 25
ALIGN_MAX_SPEED = 80
ALIGN_TIMEOUT_MS = 1000

# Torre do ultrassom: velocidade (graus/s) e angulos absolutos do motor para
# olhar para cada lado. O angulo 0 e o sensor apontado para a frente. O sinal
# depende da montagem: com o motor da torre virado para cima, +90 olha para a
# direita e -90 para a esquerda. (Na montagem anterior, com o motor virado
# para baixo, era o contrario.)
TURRET_SPEED = 800
TURRET_LEFT = -90
TURRET_RIGHT = 90

# Tempo para o ultrassom estabilizar depois que a torre para, em ms.
TURRET_SETTLE_MS = 50

# Correcao de escala do giroscopio: girando o robo a mao 360 graus (4 cantos
# de 90 graus de uma mesa), o sensor leu 353. Multiplicar a leitura por
# 360 / 353 converte o valor do sensor em graus reais.
GYRO_SCALE = 360 / 353

# Calibracao do giroscopio na largada: espera para a mao sair e o robo
# assentar, e espera para o sensor recalibrar depois de cada troca de modo, em
# ms. O robo precisa ficar totalmente parado durante todo esse tempo.
GYRO_HANDS_OFF_MS = 500
GYRO_MODE_SWITCH_MS = 300


def calibrate_gyro(gyro_sensor):
    """Recalibra o giroscopio, como a troca manual de modo no bloco: ler em
    modo velocidade e voltar para o modo angulo faz o sensor medir de novo o
    seu zero interno, eliminando o drift. Depois zera o angulo. So deve ser
    chamada com o robo parado; durante a corrida o codigo le apenas o angulo."""
    wait(GYRO_HANDS_OFF_MS)

    gyro_sensor.speed()
    wait(GYRO_MODE_SWITCH_MS)
    gyro_sensor.angle()
    wait(GYRO_MODE_SWITCH_MS)

    gyro_sensor.reset_angle(0)


def read_heading(gyro_sensor):
    """Retorna o heading do robo em graus reais: a leitura do giroscopio
    corrigida pelo fator de escala GYRO_SCALE."""
    return gyro_sensor.angle() * GYRO_SCALE


def brake(robot):
    """Para o robo freando ativamente. robot.stop() apenas solta os motores e
    o robo desliza 10-20 mm por inercia; drive(0, 0) mantem o controle de
    velocidade dos motores ativo com alvo zero, o que trava as rodas na hora."""
    robot.drive(0, 0)


def gyro_pid(robot, gyro_sensor, drive_speed, pid_params):
    """Calcula e aplica uma correcao de direcao para manter o heading medido
    pelo giroscopio igual a referencia. Retorna a variavel manipulada
    (taxa de giro, em graus/s)."""
    error = pid_params.reference - read_heading(gyro_sensor)

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
    pid_params.last_error = heading - read_heading(gyro_sensor)
    pid_params.ie_dt = 0

    while abs(pid_params.reference - read_heading(gyro_sensor)) > 1:
        turn_rate = gyro_pid(robot, gyro_sensor, 0, pid_params)
        odometry.record("turn", 0, pid_params.reference, 0, turn_rate)
        wait(SAMPLE_TIME_MS)

    brake(robot)
    wait(SETTLE_AFTER_MOVE_MS)


def drive_cell(robot, gyro_sensor, pid_params, odometry, heading, distance,
               ultrasonic_sensor, line_sensor, black_threshold,
               stop_distance, drive_speed):
    """Avanca 'distance' mm em linha reta, medidos pelos encoders, mantendo o
    heading absoluto 'heading' com o PID do giroscopio. Desacelera perto do
    fim do trecho e, por seguranca, para antes se o ultrassom medir uma parede
    a 'stop_distance' mm ou menos. Se a curva anterior parou um pouco antes ou
    depois do alvo, o PID corrige essa diferenca durante o trecho.

    Retorna True se parou na faixa preta de chegada e False se completou o
    trecho (ou parou por seguranca).

    O robo larga em cima da faixa preta, entao a faixa so passa a contar depois
    que o sensor de cor ve o chao claro pela primeira vez no trecho."""
    pid_params.reference = heading
    pid_params.last_error = heading - read_heading(gyro_sensor)
    pid_params.ie_dt = 0
    robot.reset()

    seen_floor = False

    while True:
        if line_sensor.reflection() <= black_threshold:
            if seen_floor:
                brake(robot)
                return True
        else:
            seen_floor = True

        remaining = distance - robot.distance()
        if remaining <= 0:
            brake(robot)
            return False

        # Sem obstaculo o ultrassom mede 2550 mm, que conta como caminho livre.
        front = ultrasonic_sensor.distance()
        if front <= stop_distance:
            brake(robot)
            return False

        speed = min(drive_speed,
                    max(MIN_APPROACH_SPEED, remaining * APPROACH_GAIN))
        turn_rate = gyro_pid(robot, gyro_sensor, speed, pid_params)
        odometry.record("cell", distance, pid_params.reference, speed,
                        turn_rate)
        wait(SAMPLE_TIME_MS)


def align_to_front_wall(robot, gyro_sensor, pid_params, odometry, heading,
                        ultrasonic_sensor, target_distance):
    """Centraliza o robo na celula usando a parede da frente: anda para a
    frente ou de re, mantendo o heading absoluto com o PID do giroscopio, ate
    o ultrassom medir 'target_distance' mm (dentro de ALIGN_TOLERANCE). A
    velocidade e proporcional ao erro, limitada entre ALIGN_MIN_SPEED e
    ALIGN_MAX_SPEED. Se nao chegar ao alvo em ALIGN_TIMEOUT_MS, desiste e
    segue da posicao em que estiver."""
    pid_params.reference = heading
    pid_params.last_error = heading - read_heading(gyro_sensor)
    pid_params.ie_dt = 0
    stopwatch = StopWatch()

    while stopwatch.time() < ALIGN_TIMEOUT_MS:
        error = ultrasonic_sensor.distance() - target_distance
        if abs(error) <= ALIGN_TOLERANCE:
            break

        speed = ALIGN_GAIN * error
        if speed > ALIGN_MAX_SPEED:
            speed = ALIGN_MAX_SPEED
        elif speed < -ALIGN_MAX_SPEED:
            speed = -ALIGN_MAX_SPEED
        elif 0 < speed < ALIGN_MIN_SPEED:
            speed = ALIGN_MIN_SPEED
        elif -ALIGN_MIN_SPEED < speed < 0:
            speed = -ALIGN_MIN_SPEED

        turn_rate = gyro_pid(robot, gyro_sensor, speed, pid_params)
        odometry.record("align", target_distance, pid_params.reference, speed,
                        turn_rate)
        wait(SAMPLE_TIME_MS)

    brake(robot)
    wait(SETTLE_AFTER_MOVE_MS)


def look(turret_motor, ultrasonic_sensor, turret_angle):
    """Aponta a torre para 'turret_angle', espera o ultrassom estabilizar e
    retorna a distancia medida, em mm."""
    turret_motor.run_target(TURRET_SPEED, turret_angle)
    wait(TURRET_SETTLE_MS)
    return ultrasonic_sensor.distance()


def choose_turn(turret_motor, ultrasonic_sensor, front_distance,
                open_distance):
    """Regra da mao direita: o robo prefere virar a direita, depois seguir em
    frente, depois virar a esquerda e, se tudo estiver fechado (beco sem
    saida), da meia volta. Um lado esta livre quando a leitura do ultrassom
    passa de 'open_distance' mm.

    Para ganhar tempo, a torre so olha o que a regra precisa: a direita sempre,
    e a esquerda apenas quando a direita e a frente (ja medida com a torre no
    centro) estao fechadas. No fim a torre volta a apontar para a frente.
    Retorna o giro em graus (+90 e a direita, -90 a esquerda, 0 segue em
    frente)."""
    right_distance = look(turret_motor, ultrasonic_sensor, TURRET_RIGHT)

    if right_distance > open_distance:
        turn_angle = 90
    elif front_distance > open_distance:
        turn_angle = 0
    else:
        left_distance = look(turret_motor, ultrasonic_sensor, TURRET_LEFT)
        if left_distance > open_distance:
            turn_angle = -90
        else:
            turn_angle = 180

    turret_motor.run_target(TURRET_SPEED, 0)
    return turn_angle
