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
#@cache
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


class CoralStation(enum.Enum):
    LEFT = FieldConstants.LEFT_CORAL_STATION_CENTER_POSE
    RIGHT = FieldConstants.RIGHT_CORAL_STATION_CENTER_POSE
