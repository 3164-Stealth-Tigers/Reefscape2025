import math

import commands2
from typing_extensions import Optional
from wpimath.geometry import Rotation2d, Pose2d

from constants import CoralArmConstants, ElevatorConstants, DrivingConstants, FieldConstants, RobotPhysicalConstants
from subsystems.auto_align import AutoAlign
from subsystems.coral_arm import CoralArm
from subsystems.climber import Climber
from subsystems.elevator import Elevator, SetProfiledHeightCommand
from swervepy import SwerveDrive


class Superstructure:
    def __init__(self, swerve: SwerveDrive, elevator: Elevator, coral_arm: CoralArm, climber: Climber, auto_align: AutoAlign):
        self.swerve = swerve
        self.elevator = elevator
        self.coral_arm = coral_arm
        self.climber = climber
        self.aa = auto_align

    def ready_to_score(self) -> bool:
        """Is the robot ready to release a CORAL and score?"""
        scoring_pose = self.aa.goal_pose
        return (
            # Robot is at the scoring position
            scoring_pose.translation().distance(self.swerve.pose.translation()) < DrivingConstants.MAXIMUM_POSITION_ERROR and
            abs((scoring_pose.rotation() - self.swerve.pose.rotation()).degrees()) < DrivingConstants.MAXIMUM_ANGULAR_POSITION_ERROR and
            # Robot is not moving
            abs(self.swerve.robot_relative_speeds.vx) < DrivingConstants.MAXIMUM_VELOCITY_ERROR and
            abs(self.swerve.robot_relative_speeds.vy) < DrivingConstants.MAXIMUM_VELOCITY_ERROR and
            abs(self.swerve.robot_relative_speeds.omega_dps) < DrivingConstants.MAXIMUM_ANGULAR_VELOCITY_ERROR and
            # Elevator has reached scoring height
            self.elevator.at_goal_height() and
            # Arm has reached scoring rotation
            self.coral_arm.at_goal_rotation()
        )

    def SetEndEffectorHeight(self, end_effector_height: float, angle: Optional[Rotation2d] = None):
        """
        Create a command that will move the robot's claw (end effector) to a specified height. To accomplish this,
        the elevator and arm will move simultaneously.

        :param end_effector_height: The height (in meters) to move the end effector to.
        :param angle: The counterclockwise-positive angle between the horizontal and the arm. If no angle is specified,
                      the positive angle closest to zero degrees is used.
        """

        if angle is None:
            if end_effector_height <= ElevatorConstants.MAXIMUM_CARRIAGE_HEIGHT:
                angle = Rotation2d()
            else:
                height_diff = end_effector_height - ElevatorConstants.MAXIMUM_CARRIAGE_HEIGHT
                angle = Rotation2d(math.asin(height_diff / CoralArmConstants.ARM_LENGTH))

        carriage_height = end_effector_height - (CoralArmConstants.ARM_LENGTH * angle.sin())

        if carriage_height > ElevatorConstants.MAXIMUM_CARRIAGE_HEIGHT or carriage_height < ElevatorConstants.MINIMUM_CARRIAGE_HEIGHT:
            raise Exception(f"Calculated carriage height is out of bounds: {carriage_height} meters")

        return self.coral_arm.SetAngleCommand(angle) \
            .alongWith(SetProfiledHeightCommand(carriage_height, self.elevator)) \
            .beforeStarting(commands2.PrintCommand(f"Height: {carriage_height}, Angle: {angle.degrees()}"))
