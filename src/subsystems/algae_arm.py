import commands2
import rev
import wpimath
from wpimath.geometry import Rotation2d

from constants import AlgaeArmConstants


class AlgaeArm(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.setName("Algae Arm")

        # Setup motors
        self.motor = rev.SparkFlex(AlgaeArmConstants.MOTOR_ID, rev.SparkBase.MotorType.kBrushless)
        self.absolute_encoder = self.motor.getAbsoluteEncoder()  # REV Through Bore Encoder
        self.controller = self.motor.getClosedLoopController()

        self.config()

    def config(self):
        motor_config = rev.SparkBaseConfig()

        motor_config \
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)

        motor_config.absoluteEncoder \
            .positionConversionFactor(360) \
            .velocityConversionFactor(360 / 60)

        motor_config.encoder \
            .positionConversionFactor(360 / AlgaeArmConstants.GEAR_RATIO) \
            .velocityConversionFactor(360 / AlgaeArmConstants.GEAR_RATIO / 60)

        motor_config.closedLoop \
            .setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder) \
            .pid(AlgaeArmConstants.kP, 0, 0) \
            .outputRange(-1, 1) \

        self.motor.configure(
            motor_config,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters,
        )

    def set_angle(self, angle: Rotation2d):
        """
        Command the arm to move to a specific angle. 0 degrees is considered horizontal, facing toward
        the front of the robot.

        :param angle: Counter-clockwise positive angle where 0 degrees is horizontal, facing the front of the robot.
        """
        self.controller.setReference(
            angle.degrees(), #+ CoralArmConstants.ENCODER_OFFSET,
            rev.SparkBase.ControlType.kPosition,
        )

    def angle(self) -> Rotation2d:
        degrees = self.absolute_encoder.getPosition() #- CoralArmConstants.ENCODER_OFFSET
        return Rotation2d.fromDegrees(degrees)

    def set_duty_cycle(self, output: float):
        self.motor.set(output)
