#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Color Sensor Down Program
----------------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.ev3devices import Motor, ColorSensor, TouchSensor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

from functions import gyro_pid
from models import PID_parameters
from odometry import OdometryLogger


left_motor = Motor(Port.A)
right_motor = Motor(Port.D)

line_sensor = ColorSensor(Port.S2)
touch_sensor = TouchSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S3)


robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
odometry = OdometryLogger(
    robot,
    left_motor,
    right_motor,
    line_sensor,
    touch_sensor,
    gyro_sensor,
    interval_ms=100,
)

DRIVE_SPEED = 200

BLACK_REFLECTION_THRESHOLD = 15
HEADING_KP = 2.0
HEADING_KI = 0.01
HEADING_KD = 7.5

heading_pid = PID_parameters(
    proportional_gain=HEADING_KP,
    integrative_gain=HEADING_KI,
    derivative_gain=HEADING_KD,
    reference=0,
)

square_segments = (
    (None, 500),
    (90, 1000),
    (90, 1000),
    (90, 1000),
    (90, 500),
)

gyro_sensor.reset_angle(0)
first_cycle = True

while True:
    robot.stop()
    if not first_cycle:
        while touch_sensor.pressed():
            wait(10)

    while not (touch_sensor.pressed()
               and line_sensor.reflection() <= BLACK_REFLECTION_THRESHOLD):
        wait(10)

    odometry.record("start", 0, gyro_sensor.angle(), 0, 0)

    if first_cycle:
        segments = square_segments
    else:
        segments = ((180, 0),) + square_segments

    for turn_angle, distance in segments:
        if turn_angle is not None:
            heading_pid.reference = gyro_sensor.angle() + turn_angle
            heading_pid.last_error = turn_angle
            heading_pid.ie_dt = 0

            while abs(heading_pid.reference - gyro_sensor.angle()) > 1:
                turn_rate = gyro_pid(robot, gyro_sensor, 0, heading_pid)
                odometry.record(
                    "turn", 0, heading_pid.reference, 0, turn_rate
                )
                wait(10)
            robot.stop()
            wait(100)

        if distance > 0:
            heading_pid.reference = gyro_sensor.angle()
            heading_pid.last_error = 0
            heading_pid.ie_dt = 0
            robot.reset()

            while robot.distance() < distance:
                remaining = distance - robot.distance()
                drive_speed = min(DRIVE_SPEED, max(60, remaining * 2))
                turn_rate = gyro_pid(
                    robot, gyro_sensor, drive_speed, heading_pid
                )
                odometry.record(
                    "straight", distance, heading_pid.reference,
                    drive_speed, turn_rate
                )
                wait(10)

            robot.stop()

    first_cycle = False



