import math
from wpimath.geometry import Rotation2d, Transform3d, Rotation3d

from swervepy import u


def construct_Transform3d_inches(x: float, y: float, z: float, rotation: Rotation3d) -> Transform3d:
    return Transform3d(
        x * 0.0254,
        y * 0.0254,
        z * 0.0254,
        rotation,
    )


class DrivingConstants:
    OPEN_LOOP = False
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
        "REEF_A": (2.772101, 4.002857, 0),  # (X, Y, HEADING (ROTATION))
        "REEF_B": (2.810831, 3.835833, 0),  # (X, Y, HEADING (ROTATION))
        "REEF_C": (3.462838, 2.615909, 60),  # (X, Y, HEADING (ROTATION))
        "REEF_D": (3.800886, 2.485620, 60),  # (X, Y, HEADING (ROTATION))
        "REEF_E": (5.217147, 2.432799, 120),  # (X, Y, HEADING (ROTATION))
        "REEF_F": (5.523858, 2.669971, 120),  # (X, Y, HEADING (ROTATION))
        "REEF_G": (6.163223, 3.896082, 180),  # (X, Y, HEADING (ROTATION))
        "REEF_H": (6.196424, 4.198105, 180),  # (X, Y, HEADING (ROTATION))
        "REEF_I": (5.508357, 5.400215, -120),  # (X, Y, HEADING (ROTATION))
        "REEF_J": (5.157020, 5.544599, -120),  # (X, Y, HEADING (ROTATION))
        "REEF_K": (3.790836, 5.575196, -60),  # (X, Y, HEADING (ROTATION))
        "REEF_L": (3.527338, 5.364442, -60),  # (X, Y, HEADING (ROTATION))
        "STATION_RIGHT": (1.201429, 1.207955, -126), # (X, Y, HEADING (ROTATION))
        "STATION_LEFT": (1.316851, 6.888527, 126), # (X, Y, HEADING (ROTATION))
    }

    # example usage(s): X,Y,HEADING = REEF_LEFT_CENTER
    # Y = REEF_LEFT_CENTER[2]


class ElevatorConstants:
    LEFT_MOTOR_ID = 9
    RIGHT_MOTOR_ID = 10
    INVERT_LEFT_MOTOR = False
    INVERT_RIGHT_MOTOR = False

    FEEDFORWARD_CONSTANTS = (0, 0, 0, 0)  # kS, kG, kV, kA
    kP = 13 #4
    kD = 0
    MAX_OUT_UP = 0.35

    # Hall-Effect Sensor
    LOWER_LIMIT_SWITCH_ID = 0

    # Physical measurements
    SPROCKET_PITCH_DIAMETER = (1.757 * u.inch).m_as(u.m)
    CARRIAGE_MASS = 9
    GEAR_RATIO = 5.45

    MINIMUM_CARRIAGE_HEIGHT = (30.5 * u.inch).m_as(u.m)
    MAXIMUM_CARRIAGE_HEIGHT = (80 * u.inch).m_as(u.m)  #0.560820
    LIMIT_SWITCH_HEIGHT = 0

    LEVEL_0_HEIGHT = (35 * u.inch).m_as(u.m)
    LEVEL_1_HEIGHT = (30.5 * u.inch).m_as(u.m)
    LEVEL_2_HEIGHT = (28.5 * u.inch).m_as(u.m)
    LEVEL_3_HEIGHT = (44.5 * u.inch).m_as(u.m)
    LEVEL_4_HEIGHT = (78.5 * u.inch).m_as(u.m)

    HEIGHT_TOLERANCE = 0.0254  # meters


class ClawConstants:
    # Motor configs
    MOTOR_ID = 11
    CURRENT_LIMIT_AMPS = 20

    # Sensor configs
    ToF_SENSOR_ID = 1
    ToF_MIN_DISTANCE = 101.6  # millimeters to 4 inches


class CoralArmConstants:
    MOTOR_ID = 12

    FEEDFORWARD_CONSTANTS = (0, 0, 0, 0)  # kS, kG, kV, kA
    kP = 0.02

    # Physical measurements
    ARM_LENGTH = (14 * u.inch).m_as(u.m)  # Distance from pivot point to end of arm
    ARM_MASS = 5.52
    GEAR_RATIO = 125

    MINIMUM_ANGLE = Rotation2d(-math.pi / 2)
    MAXIMUM_ANGLE = Rotation2d(math.pi / 3)

    ENCODER_OFFSET = 201.58  # degrees

    LEVEL_0_ROTATION = Rotation2d.fromDegrees(0)
    LEVEL_1_ROTATION = Rotation2d.fromDegrees(-37)
    LEVEL_2_ROTATION = Rotation2d.fromDegrees(-35)
    LEVEL_3_ROTATION = Rotation2d.fromDegrees(-35)
    LEVEL_4_ROTATION = Rotation2d.fromDegrees(-35)

    ARM_TOLERANCE = Rotation2d.fromDegrees(2)


class AlgaeArmConstants:
    MOTOR_ID = 15
    kP = 0.1

    GEAR_RATIO = 100

    STOWED_ANGLE = Rotation2d.fromDegrees(111.35)
    EXTENDED_ANGLE = Rotation2d.fromDegrees(5)


class ClimberConstants:
    LEFT_MOTOR_ID = 13
    RIGHT_MOTOR_ID = 14

    INVERT_RIGHT_MOTOR = False


class VisionConstants:
    CAMERAS = {
        "FrontTagCamera": construct_Transform3d_inches(9.5, 9, 7 + 1.75, Rotation3d.fromDegrees(0, 0, 0)),
        "BackTagCamera": construct_Transform3d_inches(8.5, -9, 22 + 1.75, Rotation3d.fromDegrees(0, 0,180)),
        "SideTagCamera": construct_Transform3d_inches(8.5, -9.5,5 + 1.75, Rotation3d.fromDegrees(0, 195, -90)),
    }

