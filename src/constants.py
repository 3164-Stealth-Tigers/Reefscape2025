class DrivingConstants:
    OPEN_LOOP = True
    FIELD_RELATIVE = True


class ElevatorConstants:
    LEFT_MOTOR_ID = 9
    RIGHT_MOTOR_ID = 10


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
