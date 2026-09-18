#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Color Sensor Down Program
----------------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, TouchSensor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

from functions import *



left_motor = Motor(Port.A)
right_motor = Motor(Port.D)

line_sensor = ColorSensor(Port.S2)
ultrasonic_sensor = UltrasonicSensor(Port.S3)
touch_sensor = TouchSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S4)

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)


BLACK = 50
WHITE = 50
threshold = (BLACK + WHITE) / 2

# Threshold da distância para o sensor ultrassônico.
DISTANCE_THRESHOLD = 200  # Distância em milímetros

# Definição da velocidade em milimetros por segundo
DRIVE_SPEED = 100

# Taxa de giro fixa para o seguidor de linha.
TURN_RATE = 30


# Start following the line endlessly.
while True:
    if touch_sensor.pressed():

        while not ultrasonic_read_bool(ultrasonic_sensor, DISTANCE_THRESHOLD):
            line_following_fixed_angle(robot, line_sensor, threshold, TURN_RATE, DRIVE_SPEED)

        gyro_turn(robot, 180)

        while not ultrasonic_read_bool(ultrasonic_sensor, DISTANCE_THRESHOLD):
            line_following_fixed_angle(robot, line_sensor, threshold, TURN_RATE, DRIVE_SPEED)

        gyro_turn(robot, 90)

        wait(3000)
        robot.stop()


    
    