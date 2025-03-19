from typing import Set

import commands2
from commands2 import Subsystem
from pathplannerlib.config import PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.trajectory import PathPlannerTrajectoryState
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds

import field
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


class DriveToScoringPosition(commands2.Command):
    def __init__(self, swerve: SwerveDrive, label: str, parameters: TrajectoryFollowerParameters):
        super().__init__()
        self.swerve = swerve
        self.label = label
        self.controller = PPHolonomicDriveController(PIDConstants(parameters.xy_kP), PIDConstants(parameters.theta_kP))

    def initialize(self):
        target_pose = field.get_robot_scoring_pose(self.label)
        self.target = PathPlannerTrajectoryState(pose=target_pose)
        self.controller.reset(self.swerve.pose, self.swerve.robot_relative_speeds)

    def execute(self):
        output = self.controller.calculateRobotRelativeSpeeds(self.swerve.pose, self.target)
        self.swerve.drive(output, DrivingConstants.OPEN_LOOP)

    def end(self, interrupted: bool):
        self.swerve.drive(ChassisSpeeds(), DrivingConstants.OPEN_LOOP)

    def getRequirements(self) -> Set[Subsystem]:
        return {self.swerve}
