import math

from wpimath.geometry import Rotation2d, Transform3d, Rotation3d, Translation2d, Transform2d, Pose2d

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
    FIELD_RELATIVE = True

    USE_READY_FOR_CLOSE = True
    USE_AUTO_SCORE = False

    CLOSE_RADIUS = (4.5 * u.ft).m_as(u.m)

    MAXIMUM_POSITION_ERROR = 0.03
    MAXIMUM_VELOCITY_ERROR = 0.03
    MAXIMUM_ANGULAR_POSITION_ERROR = 1  # degrees
    MAXIMUM_ANGULAR_VELOCITY_ERROR = 1  # degrees/sec

    REEF_WALL_TO_BUMPER_DISTANCE = (8 * u.inch).m_as(u.m)
    CORAL_STATION_WALL_TO_BUMPER_DISTANCE = (8 * u.inch).m_as(u.m)


class ElevatorConstants:
    LEFT_MOTOR_ID = 9
    RIGHT_MOTOR_ID = 10
    INVERT_LEFT_MOTOR = False
    INVERT_RIGHT_MOTOR = False

    FEEDFORWARD_CONSTANTS = (0, 0, 0, 0)  # kS, kG, kV, kA
    POSITION_kP = 4
    POSITION_kD = 3
    VELOCITY_kP = 0.1
    MAX_OUT_UP = 1 #0.35
    MAX_OUT_DOWN = -1
    MAX_VELOCITY = (150 * u.inch / u.s).m_as(u.m / u.s)
    MAX_ACCELERATION = (200 * u.inch / u.s / u.s).m_as(u.m / u.s / u.s)

    # Hall-Effect Sensor
    LOWER_LIMIT_SWITCH_ID = 0

    # Physical measurements
    SPROCKET_PITCH_DIAMETER = (1.757 * u.inch).m_as(u.m)
    CARRIAGE_MASS = 9
    GEAR_RATIO = 5.45

    MINIMUM_CARRIAGE_HEIGHT = (30.5 * u.inch).m_as(u.m)
    MAXIMUM_CARRIAGE_HEIGHT = (80 * u.inch).m_as(u.m)  #0.560820
    LIMIT_SWITCH_HEIGHT = 0

    LEVEL_0_HEIGHT = (30.5 * u.inch).m_as(u.m)
    LEVEL_1_HEIGHT = (31 * u.inch).m_as(u.m)
    LEVEL_2_HEIGHT = (31.25 * u.inch).m_as(u.m)
    LEVEL_3_HEIGHT = (46.75 * u.inch).m_as(u.m)
    LEVEL_4_HEIGHT = (78.5 * u.inch).m_as(u.m)

    HEIGHT_TOLERANCE = 0.0254  # meters


class ClawConstants:
    # Motor configs
    MOTOR_ID = 11
    CURRENT_LIMIT_AMPS = 20
    THRESHOLD_CURRENT = 15

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
    LEVEL_1_ROTATION = Rotation2d.fromDegrees(-35)
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

    ZERO_OFFSET = 0.8341073
    FORWARD_LIMIT_DEGREES = 153
    BACKWARD_LIMIT_DEGREES = 76
    LIMITS_ENABLED = True

# 10045

class VisionConstants:
    CAMERAS = {
        # Front-left swerve
        #"FLSwerveCam": construct_Transform3d_inches(13.375, 9, 8.25, Rotation3d.fromDegrees(0, -28.125, 0)
        #                                            .rotateBy(Rotation3d.fromDegrees(0, 0, 30))),
        # Right Front-facing camera
        "FRSwerveCam": construct_Transform3d_inches(9.048, -9.65, 0, Rotation3d.fromDegrees(0, 0, 0)),
        # Left Front-facing camera
        "FrontTagCamera": construct_Transform3d_inches(9.048, 9.65, 11.25, Rotation3d.fromDegrees(0, 0, 0)),
        # Back-facing camera
        #"BackTagCamera": construct_Transform3d_inches(7.952, -9.65, 25.25, Rotation3d.fromDegrees(0, 0,180)),
        # Right-facing camera, tilted 15 degrees ups
        #"SideTagCamera": construct_Transform3d_inches(8.5, -9.508, 8.669, Rotation3d.fromDegrees(0, 0, -90)
        #                                              .rotateBy(Rotation3d.fromDegrees(0, 15, 0))),
    }


class FieldConstants:
    """All positions are measured from the right side of the blue alliance wall, as defined in the FIRST layout &
    markings diagram. Unless otherwise specified, all measurements are in metres."""
    FIELD_LENGTH = 17.548
    FIELD_WIDTH = 8.052

    REEF_CENTER_TRANSLATION = Translation2d(3.302 + (2.375/2), 2.655 + (2.742/2))
    REEF_INSCRIBED_DIAMETER = 1.663  # The diameter of a circle inscribed inside the hexagon formed by the reef walls
    REEF_PIPE_TO_PIPE_DISTANCE = 0.329
    REEF_HITBOX_RADIUS = 0.960

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
    LEFT_CORAL_STATION_CENTER_POSE = Pose2d(LEFT_CORAL_STATION_CENTER_TRANSLATION, LEFT_CORAL_STATION_ROTATION)

    RIGHT_CORAL_STATION_CENTER_TRANSLATION = construct_Translation2d_inches(33.51, 25.80)
    RIGHT_CORAL_STATION_ROTATION = Rotation2d.fromDegrees(180 + 54)
    RIGHT_CORAL_STATION_CENTER_POSE = Pose2d(RIGHT_CORAL_STATION_CENTER_TRANSLATION, RIGHT_CORAL_STATION_ROTATION)

    CORAL_STATION_GROOVE_TO_GROOVE_DISTANCE = 0.2032


class RobotPhysicalConstants:
    BUMPER_LENGTH = 1.054
    BUMPER_WIDTH = 0.749

    ROBOT_RADIUS = BUMPER_LENGTH / 2

    SCORING_MECHANISM_Y_DISTANCE_TO_ROBOT_CENTER = 0#(-(0.5 + 0.25) * u.inch).m_as(u.m)
    REEF_Y_FUDGE = (0.75 * u.inch).m_as(u.m)  # Move 3/4" to the left
    SCORING_MECHANISM_RADIUS: float
