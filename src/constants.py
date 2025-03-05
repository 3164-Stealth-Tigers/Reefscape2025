import math
from wpimath.geometry import Rotation2d

from swervepy import u


class DrivingConstants:
    OPEN_LOOP = True
    FIELD_RELATIVE = True

    # Create Reef Constants (arrays/lists of floats)

    # ALGAE LOCATIONS
    ALGAE_LEFT_UPPER: list[float] = [3.801, 5.507, -33.621]  # [X, Y, HEADING]
    ALGAE_RIGHT_UPPER: list[float] = [5.446, 5.391, 125.770]  # [X, Y, HEADING]

    ALGAE_LEFT_CENTER: list[float] = [2.848, 4.189, -108.628] # [X, Y, HEADING]
    ALGAE_RIGHT_CENTER: list[float] = [6.129, 3.842, 9.926] # [X, Y, HEADING]

    ALGAE_LEFT_LOWER: list[float] = [3.531, 2.697, 32.276] # [X, Y, HEADING]
    ALGAE_RIGHT_LOWER: list[float] = [5.109, 2.486, 43.568] # [X, Y, HEADING]

    # CORAL LOCATIONS
    CORAL_LOCATIONS = {
        "REEF_A": (2.719, 4.200, 0),  # (X, Y, HEADING (ROTATION))
        "REEF_B": (2.738, 3.864, 0),  # (X, Y, HEADING (ROTATION))
        "REEF_C": (3.452, 2.605, 60),  # (X, Y, HEADING (ROTATION))
        "REEF_D": (3.764, 2.422, 60),  # (X, Y, HEADING (ROTATION))
        "REEF_E": (5.201, 2.420, 120),  # (X, Y, HEADING (ROTATION))
        "REEF_F": (5.500, 2.585, 120),  # (X, Y, HEADING (ROTATION))
        "REEF_G": (6.230, 3.857, 180),  # (X, Y, HEADING (ROTATION))
        "REEF_H": (6.244, 4.198, 180),  # (X, Y, HEADING (ROTATION))
        "REEF_I": (5.489, 5.474, -120),  # (X, Y, HEADING (ROTATION))
        "REEF_J": (5.224, 5.624, -120),  # (X, Y, HEADING (ROTATION))
        "REEF_K": (3.748, 5.627, -60),  # (X, Y, HEADING (ROTATION))
        "REEF_L": (3.479, 5.464, -60),  # (X, Y, HEADING (ROTATION))
    }

    # example usage(s): X,Y,HEADING = REEF_LEFT_CENTER
    # Y = REEF_LEFT_CENTER[2]

class ElevatorConstants:
    LEFT_MOTOR_ID = 9
    RIGHT_MOTOR_ID = 10
    INVERT_LEFT_MOTOR = False
    INVERT_RIGHT_MOTOR = True

    FEEDFORWARD_CONSTANTS = (0, 0, 0, 0)  # kS, kG, kV, kA

    # Hall-Effect Sensor
    LOWER_LIMIT_SWITCH_ID = 0

    # Physical measurements
    PULLEY_DIAMETER = (1.625 * u.inch).m_as(u.m)
    CARRIAGE_MASS = 9
    GEAR_RATIO = 5.45

    MINIMUM_CARRIAGE_HEIGHT = 0
    MAXIMUM_CARRIAGE_HEIGHT = 3
    LIMIT_SWITCH_HEIGHT = 0

    LEVEL_0_HEIGHT = 0.5
    LEVEL_1_HEIGHT = 1
    LEVEL_2_HEIGHT = 1.5
    LEVEL_3_HEIGHT = 2
    LEVEL_4_HEIGHT = 2.5


class ClawConstants:
    # Motor configs
    MOTOR_ID = 11
    CURRENT_LIMIT_AMPS = 20

    # Sensor configs
    ToF_SENSOR_ID = 1
    ToF_MIN_DISTANCE = 101.6  # millimeters to 4 inches


class ArmConstants:
    MOTOR_ID = 12
    FEEDFORWARD_CONSTANTS = (0, 0, 0, 0)  # kS, kG, kV, kA

    # Physical measurements
    ARM_LENGTH = (1 * u.ft).m_as(u.m)  # Distance from pivot point to end of arm
    ARM_MASS = 5
    GEAR_RATIO = 200

    MINIMUM_ANGLE = -math.pi /2
    MAXIMUM_ANGLE = math.pi / 3

    LEVEL_0_ROTATION = Rotation2d.fromDegrees(0)
    LEVEL_1_ROTATION = Rotation2d.fromDegrees(35)
    LEVEL_2_ROTATION = Rotation2d.fromDegrees(35)
    LEVEL_3_ROTATION = Rotation2d.fromDegrees(35)
    LEVEL_4_ROTATION = Rotation2d.fromDegrees(0)


class ClimberConstants:
    MOTOR_ID = 13
