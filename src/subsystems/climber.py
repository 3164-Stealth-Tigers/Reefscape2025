import commands2
import rev

from constants import ClimberConstants


class Climber(commands2.Subsystem):

    def __init__(self):
        super().__init__()
        self.setName("Climber")

        # Setup CIM motor
        self.motor = rev.SparkMax(ClimberConstants.MOTOR_ID, rev.SparkBase.MotorType.kBrushless)

        motor_config = rev.SparkBaseConfig()
        motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        motor_config.smartCurrentLimit(50)
        self.motor.configure(
            motor_config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters
        )

    def move_climber(self, power: float):
        self.motor.set(power)

    def stop(self):
        self.motor.set(0)

    def RaiseRobot(self):
        """Run the climber motors at a constant power, twisting the CAGE and pulling the robot up."""
        return commands2.StartEndCommand(lambda: self.move_climber(1), self.stop, self)

    def LowerRobot(self):
        """Run the climber motors at a constant power, lowering the robot."""
        return commands2.StartEndCommand(lambda: self.move_climber(-1), self.stop, self)
