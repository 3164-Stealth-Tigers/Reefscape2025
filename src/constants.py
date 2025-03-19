import math

from wpimath.geometry import Rotation2d, Transform3d, Rotation3d, Translation2d, Transform2d

from swervepy import u


def construct_Transform3d_inches(x: float, y: float, z: float, rotation: Rotation3d) -> Transform3d:
    return Transform3d(
        x * 0.0254,
        y * 0.0254,
        z * 0.0254,
        rotation,
    )


def construct_Translation2d_inches(x: float, y: float) -> Translation2d:
    return Translation2d(x * 0.0254, y * 0.0254)


class DrivingConstants:
    OPEN_LOOP = False
    FIELD_RELATIVE = False


class ElevatorConstants:
    LEFT_MOTOR_ID = 9
    RIGHT_MOTOR_ID = 10
    INVERT_LEFT_MOTOR = False
    INVERT_RIGHT_MOTOR = False

    FEEDFORWARD_CONSTANTS = (0, 0, 0, 0)  # kS, kG, kV, kA
    kP = 13

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
    LEVEL_2_HEIGHT = (31 * u.inch).m_as(u.m)
    LEVEL_3_HEIGHT = (46.5 * u.inch).m_as(u.m)
    LEVEL_4_HEIGHT = (80 * u.inch).m_as(u.m)

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
    LEVEL_4_ROTATION = Rotation2d.fromDegrees(-40)

    ARM_TOLERANCE = Rotation2d.fromDegrees(2)


class AlgaeArmConstants:
    MOTOR_ID = 15
    kP = 0.1

    GEAR_RATIO = 100

    STOWED_ANGLE = Rotation2d(-90)
    EXTENDED_ANGLE = Rotation2d()


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


class FieldConstants:
    """All positions are measured from the right side of the blue alliance wall, as defined in the FIRST layout &
    markings diagram. Unless otherwise specified, all measurements are in metres."""
    FIELD_LENGTH = 17.548
    FIELD_WIDTH = 8.052

    REEF_CENTER_TRANSLATION = Translation2d(3.302 + (2.375/2), 2.655 + (2.742/2))
    REEF_INSCRIBED_DIAMETER = 1.663  # The diameter of a circle inscribed inside the hexagon formed by the reef walls
    REEF_PIPE_TO_PIPE_DISTANCE = 0.329

    REEF_TRANSFORMATIONS = {
        "REEF_A": (1, 0),
        "REEF_B": (-1, 0),
        "REEF_C": (1, 60),
        "REEF_D": (-1, 60),
        "REEF_E": (1, 120),
        "REEF_F": (-1, 120),
        "REEF_G": (1, 180),
        "REEF_H": (-1, 180),
        "REEF_I": (1, 240),
        "REEF_J": (-1, 240),
        "REEF_K": (1, 300),
        "REEF_L": (-1, 300),
    }

    LEFT_CORAL_STATION_CENTER_TRANSLATION = construct_Translation2d_inches(33.51, 291.20)
    LEFT_CORAL_STATION_ROTATION = Rotation2d.fromDegrees(306 - 180)

    RIGHT_CORAL_STATION_CENTER_TRANSLATION = construct_Translation2d_inches(33.51, 25.80)
    RIGHT_CORAL_STATION_ROTATION = Rotation2d.fromDegrees(180 + 54)

    CORAL_STATION_GROOVE_TO_GROOVE_DISTANCE = 0.2032


class RobotPhysicalConstants:
    BUMPER_LENGTH = 1.054
    BUMPER_WIDTH = 0.749

    SCORING_MECHANISM_RELATIVE_TO_ROBOT_CENTER: Transform2d
    SCORING_MECHANISM_RADIUS: float
