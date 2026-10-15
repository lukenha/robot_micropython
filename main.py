#!/usr/bin/env pybricks-micropython

"""
Percurso em quadrado com odometria - LEGO EV3 / pybricks
--------------------------------------------------------

O robo parte do meio do primeiro lado do quadrado, percorre o quadrado no
sentido horario com giros de 90 graus e volta ao ponto de partida. Nesse ponto
ele da meia volta (180 graus) e repete o mesmo quadrado no sentido
anti-horario, parando novamente no ponto de partida. O botao e usado apenas
para iniciar: as duas voltas acontecem em sequencia, sem nova intervencao.

Durante todo o percurso o OdometryLogger grava em CSV as medidas dos sensores,
os setpoints de distancia e angulo, a variavel manipulada do PID e a pose
estimada do robo.

ATENCAO - sentido de giro: assume-se que o angulo do giroscopio cresce no
sentido horario (convencao padrao do pybricks). Isso precisa ser confirmado no
teste com o robo: se o giroscopio estiver montado ao contrario, a primeira
volta sai no anti-horario e a segunda no horario, e basta inverter os valores
de 'direction' nas chamadas de run_square().
"""

from pybricks.ev3devices import Motor, ColorSensor, TouchSensor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

from functions import run_square, turn_in_place
from models import PID_parameters
from odometry import OdometryLogger


# Geometria do robo, em mm.
WHEEL_DIAMETER = 55.5
AXLE_TRACK = 104

DRIVE_SPEED = 200
BLACK_REFLECTION_THRESHOLD = 15

HEADING_KP = 1.5 #2.0
HEADING_KI = 0.7 #0.01
HEADING_KD = 0 #7.5

CLOCKWISE = 1
COUNTERCLOCKWISE = -1

# Cada trecho do percurso e um par (angulo_de_giro, distancia_mm). O primeiro e
# o ultimo trecho tem meio lado porque o robo parte e termina no meio do
# primeiro lado do quadrado.
SQUARE_SEGMENTS = (
    (None, 500),
    (90, 1000),
    (90, 1000),
    (90, 1000),
    (90, 500),
)


left_motor = Motor(Port.A)
right_motor = Motor(Port.D)

line_sensor = ColorSensor(Port.S2)
touch_sensor = TouchSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S3)

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

# Zera as medidas com o robo parado, para que a odometria comece na origem.
gyro_sensor.reset_angle(0)
left_motor.reset_angle(0)
right_motor.reset_angle(0)
robot.reset()

# Unico acionamento manual do programa: robo posicionado sobre a linha preta e
# botao pressionado.
while not (touch_sensor.pressed()
           and line_sensor.reflection() <= BLACK_REFLECTION_THRESHOLD):
    wait(10)

odometry.record("start", 0, gyro_sensor.angle(), 0, 0)

run_square(robot, gyro_sensor, heading_pid, odometry, SQUARE_SEGMENTS,
           CLOCKWISE, DRIVE_SPEED)

turn_in_place(robot, gyro_sensor, heading_pid, odometry, 180)

# Depois da meia volta o sensor de cor fica do outro lado do eixo, entao a
# segunda volta termina pela faixa preta e nao pela distancia.
run_square(robot, gyro_sensor, heading_pid, odometry, SQUARE_SEGMENTS,
           COUNTERCLOCKWISE, DRIVE_SPEED, line_sensor,
           BLACK_REFLECTION_THRESHOLD)

robot.stop()
wait(200)  # deixa o robo parar de fato antes da ultima amostra.
odometry.record("end", 0, gyro_sensor.angle(), 0, 0)
