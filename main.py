#!/usr/bin/env pybricks-micropython

"""
Resolvedor de labirinto com odometria - LEGO EV3 / pybricks
-----------------------------------------------------------

O robo anda em linha reta, guiado pelo PID do giroscopio, ate o ultrassom
encontrar uma parede. Ali ele para, gira a torre do ultrassom para medir a
distancia ate as paredes da esquerda e da direita e gira 90 graus para o lado
mais livre. Se os dois lados estiverem fechados (beco sem saida), da meia volta
(180 graus). O percurso termina quando o sensor de cor encontra a faixa preta
de chegada. O botao e usado apenas para iniciar.

Durante todo o percurso o OdometryLogger grava em CSV as medidas dos sensores,
os setpoints de distancia e angulo, a variavel manipulada do PID e a pose
estimada do robo.

ATENCAO - sentido de giro: assume-se que o angulo do giroscopio cresce no
sentido horario (convencao padrao do pybricks), entao +90 vira para a direita e
-90 para a esquerda. Isso precisa ser confirmado no teste com o robo, assim
como o sentido da torre (TURRET_LEFT / TURRET_RIGHT em functions.py).
"""

from pybricks.ev3devices import (Motor, TouchSensor, GyroSensor,
                                 UltrasonicSensor)

# SENSOR DE LINHA DESATIVADO: o sensor de cor ainda nao esta montado, entao
# todas as mencoes a ele (line_sensor, ColorSensor, BLACK_REFLECTION_THRESHOLD)
# estao comentadas aqui, em functions.py e em odometry.py. Sem ele, a largada
# depende so do botao e a chegada nunca e detectada: o robo precisa ser parado
# manualmente (botao de voltar do EV3). Para reativar, descomentar esses trechos.
# from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

from functions import drive_until_wall, scan_sides, turn_in_place
from models import PID_parameters
from odometry import OdometryLogger


# Geometria do robo, em mm.
WHEEL_DIAMETER = 55.5
AXLE_TRACK = 104

DRIVE_SPEED = 200
# BLACK_REFLECTION_THRESHOLD = 15

HEADING_KP = 1.5 #2.0
HEADING_KI = 0.7 #0.01
HEADING_KD = 0 #7.5

# Distancia da parede da frente em que o robo para e decide o giro, em mm.
STOP_DISTANCE = 60

# Distancia lateral ate a qual o lado e considerado fechado por parede, em mm.
SIDE_OPEN_DISTANCE = 200


left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
turret_motor = Motor(Port.B)

#line_sensor = ColorSensor(Port.S2)
touch_sensor = TouchSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S3)
ultrasonic_sensor = UltrasonicSensor(Port.S4)

robot = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER,
                  axle_track=AXLE_TRACK)

odometry = OdometryLogger(
    robot,
    left_motor,
    right_motor,
    #line_sensor,
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

# Zera as medidas com o robo parado, para que a odometria comece na origem. A
# torre do ultrassom precisa estar apontada para a frente nesse momento.
gyro_sensor.reset_angle(0)
left_motor.reset_angle(0)
right_motor.reset_angle(0)
turret_motor.reset_angle(0)
robot.reset()

# Unico acionamento manual do programa: robo posicionado sobre a linha preta e
# botao pressionado. Com o sensor de linha desativado, basta o botao.
while not touch_sensor.pressed():
    # and line_sensor.reflection() <= BLACK_REFLECTION_THRESHOLD
    wait(10)

odometry.record("start", 0, gyro_sensor.angle(), 0, 0)

# Percurso no labirinto. O heading alvo e absoluto, contado desde a largada, e
# sempre multiplo de 90 graus: cada decisao soma o giro escolhido ao alvo, e
# tanto a curva quanto a reta seguinte usam esse alvo como referencia do PID.
# drive_until_wall retorna True quando encontra a faixa preta de chegada (com o
# sensor de linha desativado, nunca retorna True).
target_heading = 0

while not drive_until_wall(robot, gyro_sensor, heading_pid, odometry,
                           target_heading, ultrasonic_sensor,
                           # line_sensor, BLACK_REFLECTION_THRESHOLD,
                           STOP_DISTANCE, DRIVE_SPEED):
    left_distance, right_distance = scan_sides(turret_motor, ultrasonic_sensor)
    if (left_distance <= SIDE_OPEN_DISTANCE and right_distance <= SIDE_OPEN_DISTANCE):
        turn_angle = 180  # beco sem saida

    elif left_distance > right_distance:
        turn_angle = -90
        
    else:
        turn_angle = 90

    target_heading = target_heading + turn_angle
    turn_in_place(robot, gyro_sensor, heading_pid, odometry, target_heading)

robot.stop()
wait(200)  # deixa o robo parar de fato antes da ultima amostra.
odometry.record("end", 0, gyro_sensor.angle(), 0, 0)
