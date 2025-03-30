import commands2
import rev
from wpimath.geometry import Rotation2d
from wpiutil import SendableBuilder

from constants import ClimberConstants


class Climber(commands2.Subsystem):

    def __init__(self):
        super().__init__()
        self.setName("Climber")

        # Setup NEO motor
        self.follower_motor = rev.SparkFlex(ClimberConstants.LEFT_MOTOR_ID, rev.SparkBase.MotorType.kBrushless)
        self.leader_motor = rev.SparkFlex(ClimberConstants.RIGHT_MOTOR_ID, rev.SparkBase.MotorType.kBrushless)

        self.encoder = self.leader_motor.getAbsoluteEncoder()

        self.config()

    def config(self):
        global_config = rev.SparkBaseConfig()
        global_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        global_config.smartCurrentLimit(50)

        leader_config = rev.SparkBaseConfig()
        leader_config.apply(global_config)

        # Ready to climb position: 28.055

        leader_config.absoluteEncoder \
            .positionConversionFactor(360) \
            .velocityConversionFactor(360 / 60) \
            .zeroOffset(ClimberConstants.ZERO_OFFSET) \
            .inverted(False)

        leader_config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)

        leader_config.inverted(False)

        leader_config.softLimit \
            .forwardSoftLimit(ClimberConstants.FORWARD_LIMIT_DEGREES) \
            .reverseSoftLimit(ClimberConstants.BACKWARD_LIMIT_DEGREES) \
            .forwardSoftLimitEnabled(ClimberConstants.LIMITS_ENABLED) \
            .reverseSoftLimitEnabled(ClimberConstants.LIMITS_ENABLED)

        follower_config = rev.SparkBaseConfig()
        follower_config.apply(global_config)
        follower_config.follow(ClimberConstants.RIGHT_MOTOR_ID, ClimberConstants.INVERT_RIGHT_MOTOR)

        self.leader_motor.configure(
            leader_config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters
        )
        self.follower_motor.configure(
            follower_config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters
        )

    def move_climber(self, power: float):
        self.leader_motor.set(-power)

    def stop(self):
        self.leader_motor.set(0)

    @property
    def angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.encoder.getPosition())

    def RaiseRobot(self):
        """Run the climber motors at a constant power, twisting the CAGE and pulling the robot up."""
        return commands2.StartEndCommand(lambda: self.move_climber(1), self.stop, self)

    def LowerRobot(self):
        """Run the climber motors at a constant power, lowering the robot."""
        return commands2.StartEndCommand(lambda: self.move_climber(-1), self.stop, self)

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.addDoubleProperty("Angle", lambda: self.angle.degrees(), lambda _: None)
