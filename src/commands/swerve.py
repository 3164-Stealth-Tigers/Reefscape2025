from typing import Set

import commands2
from commands2 import Subsystem
from pathplannerlib.config import PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.trajectory import PathPlannerTrajectoryState
from wpimath.geometry import Rotation2d, Pose2d
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
