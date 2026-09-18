from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase


def line_following_proportional(robot, line_sensor, threshold, PROPORTIONAL_GAIN, DRIVE_SPEED):
    
    # Define o limite de erro para o seguidor de linha
    error = line_sensor.reflection() - threshold

    # Calcula a taxa de giro
    turn_rate = PROPORTIONAL_GAIN * error

    # Define a velocidade do robô e a taxa de giro
    robot.drive(DRIVE_SPEED, turn_rate)

def line_following_fixed_angle(robot, line_sensor, threshold, TURN_RATE, DRIVE_SPEED):
    # Seguimento de linha baseado em taxa de giro fixa
    reflection = line_sensor.reflection()

    # Corrige para o lado oposto quando a leitura ultrapassa o threshold.
    if reflection > threshold:
        turn_rate = -TURN_RATE
    else:
        turn_rate = TURN_RATE

    robot.drive(DRIVE_SPEED, turn_rate)

def ultrasonic_read_bool(ultrasonic_sensor, DISTANCE_THRESHOLD):
    # Lê a distancia do sensor ultrasônico
    distance = ultrasonic_sensor.distance()

    if distance < DISTANCE_THRESHOLD:
        return True 
    else:
        return False

def gyro_turn(robot, gyro_sensor, angle):
    # Gira o robo com base no gyro externo
    gyro_sensor.reset_angle(0)

    if angle > 0:
        while gyro_sensor.angle() < angle:
            robot.drive(0, 100)  # Gira para a direita

    else:
        while gyro_sensor.angle() > angle:
            robot.drive(0, -100)  # Gira para a esquerda

            
     