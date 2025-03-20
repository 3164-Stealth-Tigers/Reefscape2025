import enum
import math
from functools import singledispatch, cache

from wpilib import DriverStation
from wpimath.geometry import Translation2d, Rotation2d, Pose2d, Transform2d

from constants import FieldConstants, RobotPhysicalConstants, DrivingConstants


@singledispatch
def flip_alliance(arg):
    raise NotImplementedError
@flip_alliance.register
def _(translation: Translation2d) -> Translation2d:
    """Mirrors a translation across the field based on alliance color."""
    should_flip = DriverStation.getAlliance() == DriverStation.Alliance.kRed
    if should_flip:
        return Translation2d(FieldConstants.FIELD_LENGTH - translation.x, FieldConstants.FIELD_WIDTH - translation.y)
    return translation
@flip_alliance.register
def _(rotation: Rotation2d) -> Rotation2d:
    """Mirrors a rotation across the field based on alliance color."""
    should_flip = DriverStation.getAlliance() == DriverStation.Alliance.kRed
    if should_flip:
        return Rotation2d(-rotation.cos(), -rotation.sin())
    return rotation
@flip_alliance.register
def _(pose: Pose2d) -> Pose2d:
    should_flip = DriverStation.getAlliance() == DriverStation.Alliance.kRed
    if should_flip:
        return Pose2d(
            Translation2d(FieldConstants.FIELD_LENGTH - pose.translation().x,
                          FieldConstants.FIELD_WIDTH - pose.translation().y),
            Rotation2d(-pose.rotation().cos(), -pose.rotation().sin()),
        )
    return pose


# We should be able to cache the results of these functions, since the alliance remains constant throughout a match.
# This is dangerous if the alliance color switches for some reason.
@cache
def get_reef_pipe_translation(position: str):
    if position.upper() not in [chr(i) for i in range(ord('A'), ord('L') + 1)]:
        raise ValueError(f"Position out of range: {position}")

    transform_details = FieldConstants.REEF_TRANSFORMATIONS[f"REEF_{position.upper()}"]

    # Translation of pipe (either A or B) on the reef side closest to the blue driver station
    translation = FieldConstants.REEF_CENTER_TRANSLATION + Translation2d(-FieldConstants.REEF_INSCRIBED_DIAMETER/2, FieldConstants.REEF_PIPE_TO_PIPE_DISTANCE/2 * transform_details[0])

    # Rotate the translation about the center of the reef to find the translation of the correct pipe
    translation = translation.rotateAround(FieldConstants.REEF_CENTER_TRANSLATION, Rotation2d.fromDegrees(transform_details[1]))

    # If we're on red alliance, mirror the translation across the field
    return flip_alliance(translation)


@cache
def get_robot_scoring_pose(position: str):
    pipe_translation = get_reef_pipe_translation(position)

    transform_details = FieldConstants.REEF_TRANSFORMATIONS[f"REEF_{position.upper()}"]

    # Rotate such that the robot is facing the center of the reef
    rotation = Rotation2d.fromDegrees(transform_details[1])

    # Flip the robot's rotation according to the alliance color
    rotation = flip_alliance(rotation)

    # Pose centered on the reef pipe, facing the center of the reef
    robot_pose = Pose2d(pipe_translation, rotation)

    # Offset the pose such that the front of the bumper touches the reef wall
    robot_pose = robot_pose.transformBy(
        Transform2d(-RobotPhysicalConstants.BUMPER_LENGTH / 2, 0, 0)).transformBy(
        Transform2d(-DrivingConstants.REEF_WALL_TO_BUMPER_DISTANCE, 0, 0)
    )
    return robot_pose


class CoralStation(enum.Enum):
    LEFT = FieldConstants.LEFT_CORAL_STATION_CENTER_POSE
    RIGHT = FieldConstants.RIGHT_CORAL_STATION_CENTER_POSE


@cache
def get_robot_intake_pose(station: CoralStation):
    robot_pose = flip_alliance(station.value)

    robot_pose = robot_pose.transformBy(
        Transform2d(-RobotPhysicalConstants.BUMPER_LENGTH / 2, 0, 0)).transformBy(
        Transform2d(-DrivingConstants.CORAL_STATION_WALL_TO_BUMPER_DISTANCE, 0, 0)
    )
    return robot_pose


def will_collide(a_initial: Translation2d, a_final: Translation2d, b: Translation2d, radius_a: float, radius_b: float) -> bool:
    """
    Checks if a circle A moving along a linear path will collide with another circle B.

    For an explanation of the function's inner-workings, check ``docs/collision_detection.md``.

    :param a_initial: The initial position of circle A.
    :param a_final: The final position of circle A.
    :param b: The position of circle B.
    :param radius_a: The radius of circle A.
    :param radius_b: The radius of circle B.
    """

    # Setup a quadratic
    A = (a_final.x - a_initial.x) ** 2 + (a_final.y - a_initial.y) ** 2
    B = 2 * ( (a_initial.x - b.x) * (a_final.x - a_initial.x) + (a_initial.y - b.y) * (a_final.y - a_initial.y) )
    C = (a_initial.x - b.x) ** 2 + (a_initial.y - b.y) ** 2 - (radius_a + radius_b) ** 2
    discriminant = B ** 2 - 4 * A * C

    # Check if there is at least one solution
    if discriminant >= 0:
        # Solve the quadratic equation
        t_1 = (-B + math.sqrt(discriminant)) / (2 * A)
        t_2 = (-B - math.sqrt(discriminant)) / (2 * A)

        # Check if the roots are within [0, 1]
        if 0 <= t_1 <= 1 and 0 <= t_2 <= 1:
            return True

    # If no solutions exist or the solutions are not within [0, 1], then no collision occurs
    return False
