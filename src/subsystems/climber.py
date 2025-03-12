import commands2
import rev

from constants import ClimberConstants


class Climber(commands2.Subsystem):

    def __init__(self):
        super().__init__()
        self.setName("Climber")

        # Setup NEO motor
        self.leader_motor = rev.SparkFlex(ClimberConstants.LEFT_MOTOR_ID, rev.SparkBase.MotorType.kBrushless)
        self.follower_motor = rev.SparkFlex(ClimberConstants.RIGHT_MOTOR_ID, rev.SparkBase.MotorType.kBrushless)

        global_config = rev.SparkBaseConfig()
        global_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        global_config.smartCurrentLimit(50)

        follower_config = rev.SparkBaseConfig()
        follower_config.apply(global_config)
        follower_config.follow(ClimberConstants.LEFT_MOTOR_ID, ClimberConstants.INVERT_RIGHT_MOTOR)

        self.leader_motor.configure(
            global_config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters
        )
        self.follower_motor.configure(
            follower_config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters
        )

    def move_climber(self, power: float):
        self.leader_motor.set(power)

    def stop(self):
        self.leader_motor.set(0)

    def RaiseRobot(self):
        """Run the climber motors at a constant power, twisting the CAGE and pulling the robot up."""
        return commands2.StartEndCommand(lambda: self.move_climber(1), self.stop, self)

    def LowerRobot(self):
        """Run the climber motors at a constant power, lowering the robot."""
        return commands2.StartEndCommand(lambda: self.move_climber(-1), self.stop, self)
