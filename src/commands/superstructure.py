import math

import commands2
from typing_extensions import Optional
from wpimath.geometry import Rotation2d

from constants import CoralArmConstants, ElevatorConstants
from subsystems.coral_arm import CoralArm
from subsystems.climber import Climber
from subsystems.elevator import Elevator


class Superstructure:
    def __init__(self, elevator: Elevator, coral_arm: CoralArm, climber: Climber):
        self.elevator = elevator
        self.coral_arm = coral_arm
        self.climber = climber

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
            raise Exception("Calculated carriage height is out of bounds.")

        return self.coral_arm.SetAngleCommand(angle) \
            .alongWith(self.elevator.SetHeightCommand(carriage_height)) \
            .beforeStarting(commands2.PrintCommand(f"Height: {carriage_height}, Angle: {angle.degrees()}"))
