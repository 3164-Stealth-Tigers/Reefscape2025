import math

from swervepy import u


class DrivingConstants:
    OPEN_LOOP = True
    FIELD_RELATIVE = True


class ElevatorConstants:
    LEFT_MOTOR_ID = 9
    RIGHT_MOTOR_ID = 10
    INVERT_LEFT_MOTOR = False
    INVERT_RIGHT_MOTOR = True

    LOWER_LIMIT_SWITCH_ID = 0

    # Physical measurements
    PULLEY_DIAMETER = (1.625 * u.inch).m_as(u.m)
    CARRIAGE_MASS = 9
    GEAR_RATIO = 5.45

    MINIMUM_CARRIAGE_HEIGHT = 0
    MAXIMUM_CARRIAGE_HEIGHT = 3


class ClawConstants:
    # Motor configs
    MOTOR_ID = 11
    CURRENT_LIMIT_AMPS = 20

    # Sensor configs
    ToF_SENSOR_ID = 1
    ToF_MIN_DISTANCE = 1  # millimeters


class ArmConstants:
    MOTOR_ID = 12
    FEEDFORWARD_CONSTANTS = (0, 0, 0, 0)  # kS, kG, kV, kA

    # Physical measurements
    ARM_LENGTH = (1 * u.ft).m_as(u.m)  # Distance from pivot point to end of arm
    ARM_MASS = 5
    GEAR_RATIO = 200

    MINIMUM_ANGLE = -math.pi / 3
    MAXIMUM_ANGLE = math.pi / 3

class ClimberConstants:
    MOTOR_ID = 13
