#!/usr/bin/env pybricks-micropython

"""
Resolvedor de labirinto com odometria - LEGO EV3 / pybricks
-----------------------------------------------------------

O robo percorre o labirinto celula a celula (cada celula tem ~250 mm). Em
cada celula ele:
  1. avanca ate o centro, medindo a distancia pelos encoders e mantendo a
     direcao com o PID do giroscopio;
  2. se houver parede a frente, se alinha por ela com o ultrassom, corrigindo
     o erro acumulado dos encoders;
  3. gira a torre do ultrassom para medir a esquerda e a direita;
  4. decide pela regra da mao direita (direita, frente, esquerda ou meia
     volta) e gira para o heading escolhido.
O percurso termina quando o sensor de cor encontra a faixa preta de chegada.
O botao e usado apenas para iniciar.

Durante todo o percurso o OdometryLogger grava em CSV as medidas dos sensores,
os setpoints de distancia e angulo, a variavel manipulada do PID e a pose
estimada do robo.

ATENCAO - sentido de giro: assume-se que o angulo do giroscopio cresce no
sentido horario (convencao padrao do pybricks), entao +90 vira para a direita e
-90 para a esquerda. Isso precisa ser confirmado no teste com o robo, assim
como o sentido da torre (TURRET_LEFT / TURRET_RIGHT em functions.py).
"""

from pybricks.ev3devices import (Motor, ColorSensor, TouchSensor, GyroSensor,
                                 UltrasonicSensor)
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

from functions import (align_to_front_wall, calibrate_gyro, choose_turn,
                       drive_cell, read_heading, turn_in_place)
from models import PID_parameters
from odometry import OdometryLogger


# Geometria do robo, em mm. AXLE_TRACK e a distancia entre os centros das
# rodas: 146 mm entre as extremidades externas menos 28 mm de largura do pneu.
WHEEL_DIAMETER = 55.5
AXLE_TRACK = 118

DRIVE_SPEED = 200
BLACK_REFLECTION_THRESHOLD = 15

# KP subiu de 1.5 para 2.0 para tirar o erro de 1-2 graus que sobrava nas retas.
# Para reverter, voltar para 1.5.
HEADING_KP = 2.0
HEADING_KI = 0.7 #0.01
HEADING_KD = 0 #7.5

# Parada de seguranca: se o ultrassom medir uma parede a essa distancia
# durante um avanco, o robo para antes de completar a celula, em mm.
STOP_DISTANCE = 35

# Leitura do ultrassom acima da qual um lado (ou a frente) e considerado livre,
# em mm.
OPEN_DISTANCE = 200

# Geometria do labirinto, em mm. Valores ESTIMADOS, a medir no robo:
# - CELL_SIZE: lado de uma celula do labirinto;
# - FIRST_CELL_DISTANCE: quanto o robo anda da largada ate a posicao de parada
#   da primeira celula. Na largada (faixa preta sob o sensor de cor) o
#   ultrassom fica a ~115 mm da parede da primeira celula; andando
#   115 - FRONT_CENTER_DISTANCE = 80 mm, ele para na mesma posicao relativa
#   a celula que nas demais (ultrassom a 35 mm da parede);
# - FRONT_CENTER_DISTANCE: leitura do ultrassom com o robo no centro de uma
#   celula e parede a frente. O ultrassom do EV3 so mede com confianca a
#   partir de ~30 mm, entao 35 mm (com a tolerancia de 5 mm do alinhamento)
#   e o mais perto possivel da parede.
CELL_SIZE = 250
FIRST_CELL_DISTANCE = 80
FRONT_CENTER_DISTANCE = 35


# Na montagem atual, o motor da roda esquerda esta na porta D e o da direita
# na porta A.
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)
turret_motor = Motor(Port.B)

line_sensor = ColorSensor(Port.S2)
touch_sensor = TouchSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S3)
ultrasonic_sensor = UltrasonicSensor(Port.S4)

robot = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER,
                  axle_track=AXLE_TRACK)

odometry = OdometryLogger(
    robot,
    left_motor,
    right_motor,
    line_sensor,
    touch_sensor,
    gyro_sensor,
    wheel_diameter=WHEEL_DIAMETER,
    axle_track=AXLE_TRACK,
    interval_ms=100,
)

heading_pid = PID_parameters(
    proportional_gain=HEADING_KP,
    integrative_gain=HEADING_KI,
    derivative_gain=HEADING_KD,
    reference=0,
)

# Unico acionamento manual do programa: robo posicionado sobre a linha preta e
# botao pressionado.
while not (touch_sensor.pressed()
           and line_sensor.reflection() <= BLACK_REFLECTION_THRESHOLD):
    wait(10)

# Depois do botao (que mexe o robo ao ser apertado), recalibra o giroscopio e
# zera as medidas com o robo parado, para que a odometria comece na origem. A
# torre do ultrassom precisa estar apontada para a frente nesse momento.
calibrate_gyro(gyro_sensor)
left_motor.reset_angle(0)
right_motor.reset_angle(0)
turret_motor.reset_angle(0)
robot.reset()

odometry.record("start", 0, read_heading(gyro_sensor), 0, 0)

# Percurso no labirinto, celula a celula. O heading alvo e absoluto, contado
# desde a largada, e sempre multiplo de 90 graus: cada decisao soma o giro
# escolhido ao alvo, e tanto a curva quanto o avanco seguinte usam esse alvo
# como referencia do PID. drive_cell retorna True quando encontra a faixa preta
# de chegada.
target_heading = 0

reached_exit = drive_cell(robot, gyro_sensor, heading_pid, odometry,
                          target_heading, FIRST_CELL_DISTANCE,
                          ultrasonic_sensor, line_sensor,
                          BLACK_REFLECTION_THRESHOLD, STOP_DISTANCE,
                          DRIVE_SPEED)

while not reached_exit:
    # Com parede a frente, o robo se alinha por ela antes de olhar os lados,
    # para que as leituras laterais sejam feitas ja no centro da celula.
    front_distance = ultrasonic_sensor.distance()
    if front_distance <= OPEN_DISTANCE:
        align_to_front_wall(robot, gyro_sensor, heading_pid, odometry,
                            target_heading, ultrasonic_sensor,
                            FRONT_CENTER_DISTANCE)

    turn_angle = choose_turn(turret_motor, ultrasonic_sensor, front_distance,
                             OPEN_DISTANCE)

    if turn_angle != 0:
        target_heading = target_heading + turn_angle
        turn_in_place(robot, gyro_sensor, heading_pid, odometry,
                      target_heading)

    reached_exit = drive_cell(robot, gyro_sensor, heading_pid, odometry,
                              target_heading, CELL_SIZE, ultrasonic_sensor,
                              line_sensor, BLACK_REFLECTION_THRESHOLD,
                              STOP_DISTANCE, DRIVE_SPEED)

robot.stop()
wait(200)  # deixa o robo parar de fato antes da ultima amostra.
odometry.record("end", 0, read_heading(gyro_sensor), 0, 0)
