import commands2
import rev
import playingwithfusion as pwf
from commands2 import Command

from constants import ClawConstants


class Claw(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.setName("Claw")

        # NEO 550 motor to spin intake wheels
        self.motor = rev.SparkMax(ClawConstants.MOTOR_ID, rev.SparkMax.MotorType.kBrushless)

        motor_config = rev.SparkBaseConfig()
        motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)  # Prevent CORAL from getting stuck in the claw
        motor_config.smartCurrentLimit(20)
        motor_config.inverted(True)
        self.motor.configure(
            motor_config, rev.SparkBase.ResetMode.kResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters
        )

        # Range sensor to detect coral possession
        self.distance_sensor = pwf.TimeOfFlight(ClawConstants.ToF_SENSOR_ID)

        #distance = self.distance_sensor.getRange()

    def intake(self):
        """Run the intake motors at a constant power, pulling CORAL into the claw."""
        self.motor.set(0.2)

    def outtake(self):
        """Run the intake motors at a constant power, pushing CORAL out of the claw."""
        self.motor.set(-0.5)

    def stop(self):
        """Stop running the intake motors."""
        self.motor.set(0)

    def has_possession(self) -> bool:
        """Return whether the claw is currently holding CORAL."""
        # If the distance reported by the sensor is less than a certain known distance,
        # a CORAL is sitting above the sensor; therefore, the claw has possession

        return self.distance_sensor.getRange() <= ClawConstants.ToF_MIN_DISTANCE


        # If the intake motors are stalled, the claw has possession of a game piece
         #return self.motor.getOutputCurrent() > (ClawConstants.CURRENT_LIMIT_AMPS - 2)

    def IntakeCommand(self):
        """Pulls the CORAL into the claw. This command will not end on its own; it must be interrupted by the user."""
        return commands2.StartEndCommand(self.intake, self.stop, self)

    def OuttakeCommand(self):
        """Ejects the CORAL from the claw. This command ends after the CORAL has been ejected."""
        return commands2.StartEndCommand(self.outtake, self.stop, self)#.onlyWhile(self.has_possession)
