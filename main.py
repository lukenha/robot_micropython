#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Color Sensor Down Program
----------------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.ev3devices import Motor, ColorSensor, TouchSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

from functions import *
from models import PID_parameters


left_motor = Motor(Port.A)
right_motor = Motor(Port.D)

line_sensor = ColorSensor(Port.S2)
touch_sensor = TouchSensor(Port.S1)


robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

DRIVE_SPEED = 150

# Parâmetros PID para o seguidor de linha
pid_params = PID_parameters(
    proportional_gain=5.0,
    integrative_gain=0.001,
    derivative_gain=1,
    reference=50
)

# Start and stop line following with each button press.
running = False
button_was_pressed = False

while True:
    button_is_pressed = touch_sensor.pressed()

    if button_is_pressed and not button_was_pressed:
        running = not running
        if not running:
            robot.stop()

    if running:
        pid_params.last_error = line_following_pid(robot, line_sensor, DRIVE_SPEED, pid_params)
    button_was_pressed = button_is_pressed
    wait(10)
    


