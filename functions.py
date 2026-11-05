from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from urandom import random


def drive_straight(robot, DRIVE_SPEED):
    
    # Define a velocidade do robô e a taxa de giro
    robot.drive(DRIVE_SPEED, 0)


def random_turn(robot):
    # Escolhe um valor aleatorio entre -120 e +120 (sem ser 0) e gira o robo

    random_angle = random.randint(-120, 120)
    while random_angle == 0:
        random_angle = random.randint(-120, 120)

    robot.drive(0, random_angle)


def ultrasonic_read_bool(ultrasonic_sensor, DISTANCE_THRESHOLD):
    # Lê a distancia do sensor ultrasônico
    distance = ultrasonic_sensor.distance()

    if distance < DISTANCE_THRESHOLD:
        return True 
    else:
        return False


