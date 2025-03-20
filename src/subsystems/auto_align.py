import enum
import math
from functools import cache
from typing import Set

import commands2
import wpilib
from commands2 import Subsystem, Command
from pathplannerlib.config import PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.trajectory import PathPlannerTrajectoryState
from wpimath.geometry import Translation2d, Transform2d, Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds

from constants import RobotPhysicalConstants, FieldConstants, DrivingConstants
from field import flip_alliance, get_reef_pipe_translation, CoralStation
from swervepy import SwerveDrive, TrajectoryFollowerParameters


class AutoAlign(Subsystem):

    def __init__(self, swerve: SwerveDrive):
        super().__init__()

        self.swerve = swerve
        self.goal_pose = Pose2d()

    def ready_for_close(self) -> bool:
        """Is the robot close enough to its scoring position for close actions (e.g., raising the elevator)?"""
        ready = self.goal_pose.translation().distance(self.swerve.pose.translation()) < DrivingConstants.CLOSE_RADIUS
        wpilib.SmartDashboard.putBoolean("AutoAlign/ReadyForClose", ready)
        return ready

    def will_collide_with_reef(self):
        return self.will_collide(self.swerve.pose.translation(), self.goal_pose.translation(),
                            flip_alliance(FieldConstants.REEF_CENTER_TRANSLATION),
                            RobotPhysicalConstants.ROBOT_RADIUS, FieldConstants.REEF_HITBOX_RADIUS)

    @staticmethod
    #@cache
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
            Transform2d(0, -RobotPhysicalConstants.SCORING_MECHANISM_Y_DISTANCE_TO_ROBOT_CENTER, 0)).transformBy(
            Transform2d(-RobotPhysicalConstants.BUMPER_LENGTH / 2, 0, 0)).transformBy(
            Transform2d(-DrivingConstants.REEF_WALL_TO_BUMPER_DISTANCE, 0, 0)
        )
        return robot_pose

    @staticmethod
    #@cache
    def get_robot_intake_pose(station: CoralStation):
        robot_pose = flip_alliance(station.value)

        robot_pose = robot_pose.transformBy(
            Transform2d(-RobotPhysicalConstants.BUMPER_LENGTH / 2, 0, 0)).transformBy(
            Transform2d(-DrivingConstants.CORAL_STATION_WALL_TO_BUMPER_DISTANCE, 0, 0)
        )
        return robot_pose

    @staticmethod
    def will_collide(a_initial: Translation2d, a_final: Translation2d, b: Translation2d, radius_a: float,
                     radius_b: float) -> bool:
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
        B = 2 * ((a_initial.x - b.x) * (a_final.x - a_initial.x) + (a_initial.y - b.y) * (a_final.y - a_initial.y))
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

    def close_command(self, cmd: Command) -> Command:
        return commands2.StartEndCommand(lambda: print(f"{cmd.getName()} waiting for close to reef."),
                                         lambda: None, *cmd.requirements) \
            .until(self.ready_for_close) \
            .andThen(cmd)

    def periodic(self) -> None:
        field = self.swerve.field
        field.getObject("GoalPose").setPose(self.goal_pose)


class DriveToScoringPosition(commands2.Command):
    def __init__(self, auto_align: AutoAlign, label: str, parameters: TrajectoryFollowerParameters):
        super().__init__()
        self.aa = auto_align
        self.swerve = auto_align.swerve
        self.label = label
        self.controller = PPHolonomicDriveController(PIDConstants(parameters.xy_kP), PIDConstants(parameters.theta_kP))

    def initialize(self):
        target_pose = AutoAlign.get_robot_scoring_pose(self.label)
        self.aa.goal_pose = target_pose
        self.target = PathPlannerTrajectoryState(pose=target_pose)
        self.controller.reset(self.swerve.pose, self.swerve.robot_relative_speeds)

    def execute(self):
        output = self.controller.calculateRobotRelativeSpeeds(self.swerve.pose, self.target)
        self.swerve.drive(output, DrivingConstants.OPEN_LOOP)

    def end(self, interrupted: bool):
        self.swerve.drive(ChassisSpeeds(), DrivingConstants.OPEN_LOOP)

    def getRequirements(self) -> Set[Subsystem]:
        return {self, self.swerve}

