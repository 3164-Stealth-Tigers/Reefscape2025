import commands2
import rev
import wpimath.controller
from wpimath.geometry import Rotation2d

from constants import ArmConstants


class Arm(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.setName("Arm")

        # Setup motors
        self.motor = rev.SparkFlex(ArmConstants.MOTOR_ID, rev.SparkBase.MotorType.kBrushless)
        self.absolute_encoder = self.motor.getAbsoluteEncoder()  # REV Through Bore Encoder
        self.controller = self.motor.getClosedLoopController()

        self.feedforward = wpimath.controller.ArmFeedforward(*ArmConstants.FEEDFORWARD_CONSTANTS)

    def set_angle(self, angle: Rotation2d):
        """
        Command the arm to move to a specific angle. 0 degrees is considered horizontal, facing toward
        the front of the robot.

        :param angle: Counter-clockwise positive angle where 0 degrees is horizontal, facing the front of the robot.
        """
        ff = self.feedforward.calculate(self.absolute_encoder.getPosition(), self.absolute_encoder.getVelocity())
        self.controller.setReference(
            angle.degrees(),
            rev.SparkBase.ControlType.kPosition,
            arbFeedforward=ff,
            arbFFUnits=rev.SparkClosedLoopController.ArbFFUnits.kVoltage,
        )

    def angle(self) -> Rotation2d:
        degrees = self.absolute_encoder.getPosition()
        return Rotation2d.fromDegrees(degrees)