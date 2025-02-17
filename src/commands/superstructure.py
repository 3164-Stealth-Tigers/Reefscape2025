import commands2
from typing_extensions import Optional
from wpimath.geometry import Rotation2d

from constants import ArmConstants, ElevatorConstants
from subsystems.arm import Arm
from subsystems.climber import Climber
from subsystems.elevator import Elevator


class Superstructure:
    def __init__(self, elevator: Elevator, arm: Arm, climber: Climber):
        self.elevator = elevator
        self.arm = arm
        self.climber = climber

    def SetEndEffectorHeight(self, end_effector_height: float, angle: Optional[Rotation2d] = Rotation2d(0)):
        carriage_height = end_effector_height - (ArmConstants.ARM_LENGTH * angle.sin())

        if carriage_height > ElevatorConstants.MAXIMUM_CARRIAGE_HEIGHT or carriage_height < ElevatorConstants.MINIMUM_CARRIAGE_HEIGHT:
            raise Exception("Calculated carriage height is out of bounds.")

        return commands2.RunCommand(lambda: self.arm.set_angle(angle), self.arm).alongWith(
            commands2.RunCommand(lambda: self.elevator.set_height(carriage_height), self.elevator)
        )
