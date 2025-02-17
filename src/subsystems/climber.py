import commands2
import rev

from constants import ClimberConstants


class Climber(commands2.Subsystem):

    def __init__(self):
        super().__init__()
        self.setName("Climber")

        # Setup CIM motor
        self.motor = rev.SparkMax(ClimberConstants.MOTOR_ID, rev.SparkBase.MotorType.kBrushed)

        motor_config = rev.SparkBaseConfig()
        motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self.motor.configure(
            motor_config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters
        )

    def raise_robot(self):
        """Run the climber motors at a constant power, twisting the CAGE and pulling the robot up."""
        self.motor.set(1)

    def lower_robot(self):
        """Run the climber motors at a constant power, lowering the robot."""
        self.motor.set(-1)
