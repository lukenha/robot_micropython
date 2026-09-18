from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

from models import PID_parameters

def line_following_pid(robot, line_sensor, drive_speed, pid_params):
    
    # Define o limite de erro para o seguidor de linha
    error = line_sensor.reflection() - pid_params.reference
    de_dt = error - pid_params.last_error
    ie_dt_temp = pid_params.ie_dt + error

    proportional_term = pid_params.proportional_gain * error
    integrative_term = pid_params.integrative_gain * ie_dt_temp
    derivative_term = pid_params.derivative_gain * de_dt

    control_action_raw = proportional_term + integrative_term + derivative_term

    if control_action_raw > 100:
        control_action = 100 
    elif control_action_raw < -100:
        control_action = -100
    else:
        control_action = control_action_raw
        pid_params.ie_dt = ie_dt_temp

    # The robot's positive turn direction is opposite to the correction needed here.
    robot.drive(drive_speed, -control_action)

    pid_params.last_error = error
    return pid_params.last_error


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

            
     