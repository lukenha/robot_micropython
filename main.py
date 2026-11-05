#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Color Sensor Down Program
----------------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.ev3devices import Motor, UltrasonicSensor 
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile
from pybricks.hubs import EV3Brick

from functions import *
from server import *

brick = EV3Brick()
soundfile = SoundFile()

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)

ultrasonic_sensor = UltrasonicSensor(Port.S3)

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Threshold da distância para o sensor ultrassônico.
DISTANCE_THRESHOLD = 300  # Distância em milímetros

# Definição da velocidade em milimetros por segundo
DRIVE_SPEED = 100

# Taxa de giro fixa para o seguidor de linha.
TURN_RATE = 30

mail_box = server_setup()

iteration = 0

while iteration != 3:

    # Loop principal

    # Anda reto pra frente ate encontrar um obstaculo
    # Depois, gira aleatoriamente e envia uma mensagem para o cliente 1
    while not ultrasonic_read_bool(ultrasonic_sensor, DISTANCE_THRESHOLD):
        drive_straight(robot, DRIVE_SPEED)
    else:
        brick.speaker.play_file(soundfile.UH_OH)
        random_turn(robot)
        server_send(mail_box, "client_1: go")
        brick.speaker.play_file(soundfile.GO)

    # Espera client 1 terminar e inicia client 2
    while server_receive(mail_box) != "client_1: done":
        wait(100)
    else:
        brick.speaker.play_file(soundfile.GOOD_JOB)
        server_send(mail_box, "client_2: go")
        brick.speaker.play_file(soundfile.GO)

    # Espera client 2 terminar e proxima iteracao
    while server_receive(mail_box) != "client_2: done":
        wait(100)
    else:
        brick.speaker.play_file(soundfile.GOOD_JOB)
        iteration += 1


