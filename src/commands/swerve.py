from typing import Set

import commands2
from commands2 import Subsystem
from pathplannerlib.config import PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.trajectory import PathPlannerTrajectoryState
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds

from constants import DrivingConstants
from swervepy import SwerveDrive, TrajectoryFollowerParameters


def SkiStopCommand(swerve: SwerveDrive):
    # fmt: off
    angles = (
        45, 315,  # Front Left, Front Right
        315, 45,  # Back Left, Back Right
    )
    # fmt: on

    states = tuple(SwerveModuleState(0, Rotation2d.fromDegrees(angle)) for angle in angles)

    return commands2.RunCommand(lambda: swerve.desire_module_states(states), swerve)


class DriveToPoseCommand(commands2.Command):
    def __init__(self, swerve: SwerveDrive, target: Pose2d, parameters: TrajectoryFollowerParameters):
        """
        A Command that drive the robot to a given pose on the field. This Command may be run continuously to keep the
        robot "locked" to target pose while scoring or intaking.

        In the field coordinate system, the +X direction points toward the Red Alliance Driver Station, and the +Y
        direction is rotated 90 degrees counterclockwise from the +X direction. Rotations are counterclockwise-positive.

        A diagram may be found in the docs folder or
        at this link: https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html.

        :param swerve: The swerve drive subsystem.
        :param target: The desired pose for the robot in the field coordinate system.
        :param parameters: Parameters for tracking a pose autonomously.
        """

        super().__init__()
        self.swerve = swerve
        self.target = PathPlannerTrajectoryState(pose=target)
        self.controller = PPHolonomicDriveController(PIDConstants(parameters.xy_kP), PIDConstants(parameters.theta_kP))

    def initialize(self):
        self.controller.reset(self.swerve.pose, self.swerve.robot_relative_speeds)

    def execute(self):
        output = self.controller.calculateRobotRelativeSpeeds(self.swerve.pose, self.target)
        self.swerve.drive(output, DrivingConstants.OPEN_LOOP)

    def end(self, interrupted: bool):
        self.swerve.drive(ChassisSpeeds(), DrivingConstants.OPEN_LOOP)

    def getRequirements(self) -> Set[Subsystem]:
        return {self.swerve}


class DriveDistanceCommand(commands2.Command):
    def __init__(self, swerve: SwerveDrive, x_speed: float, y_speed: float, distance: float):
        """
        Drive the robot in a straight line for a specified distance.

        :param swerve: The swerve drive subsystem.
        :param x_speed: The speed (in meters/sec) to move in the x-direction across the field. A positive number will move toward the Red Driver Station, and a negative number will move toward the Blue Driver Station.
        :param y_speed: The speed (in meters/sec) to move in the y-direction. A positive number will move up the field, and a negative number will move down the field.
        :param distance: The straight-line distance (in meters) to drive. This number should be positive.
        """

        super().__init__()
        self.swerve = swerve
        self.speeds = Translation2d(x_speed, y_speed)
        self.distance = distance

    def initialize(self):
        self.initial_translation = self.swerve.pose.translation()

    def execute(self):
        self.swerve.drive(self.speeds, 0, False, False)

    def end(self, interrupted: bool):
        self.swerve.drive(Translation2d(0, 0), 0, False, False)

    def isFinished(self) -> bool:
        return self.swerve.pose.translation().distance(self.initial_translation) >= self.distance

    def getRequirements(self) -> Set[Subsystem]:
        return {self.swerve}
